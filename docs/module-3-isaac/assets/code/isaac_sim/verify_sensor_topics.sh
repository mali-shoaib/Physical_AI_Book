#!/bin/bash
###############################################################################
# Verify ROS 2 topics from Isaac Sim are publishing correctly
#
# This script checks that:
# 1. Required ROS 2 topics exist
# 2. Topics are publishing at expected frequencies
# 3. Message types are correct
#
# Usage:
#   ./verify_sensor_topics.sh
#
# Requirements:
#   - ROS 2 Humble sourced
#   - Isaac Sim running with ROS 2 bridge enabled
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS 2 environment not sourced!${NC}"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}ROS 2 Distribution: $ROS_DISTRO${NC}"
echo -e "${GREEN}ROS Domain ID: ${ROS_DOMAIN_ID:-0}${NC}"
echo ""

# Expected topics
EXPECTED_TOPICS=(
    "/left/image_raw"
    "/right/image_raw"
    "/clock"
)

# Topic type expectations
declare -A TOPIC_TYPES
TOPIC_TYPES["/left/image_raw"]="sensor_msgs/msg/Image"
TOPIC_TYPES["/right/image_raw"]="sensor_msgs/msg/Image"
TOPIC_TYPES["/clock"]="rosgraph_msgs/msg/Clock"

# Frequency expectations (Hz)
declare -A TOPIC_FREQ
TOPIC_FREQ["/left/image_raw"]=60
TOPIC_FREQ["/right/image_raw"]=60
TOPIC_FREQ["/clock"]=60

echo "=================================================="
echo "ROS 2 Topic Verification"
echo "=================================================="
echo ""

# Get list of available topics
echo "Fetching topic list..."
ALL_TOPICS=$(ros2 topic list 2>/dev/null)

if [ -z "$ALL_TOPICS" ]; then
    echo -e "${RED}✗ No topics found! Is Isaac Sim running with ROS 2 bridge?${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS 2 is running${NC}"
echo ""

# Check each expected topic
PASS_COUNT=0
FAIL_COUNT=0

for topic in "${EXPECTED_TOPICS[@]}"; do
    echo "--------------------------------------------------"
    echo "Checking: $topic"
    echo "--------------------------------------------------"
    
    # Check if topic exists
    if echo "$ALL_TOPICS" | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓ Topic exists${NC}"
        
        # Check topic type
        ACTUAL_TYPE=$(ros2 topic type "$topic" 2>/dev/null)
        EXPECTED_TYPE="${TOPIC_TYPES[$topic]}"
        
        if [ "$ACTUAL_TYPE" == "$EXPECTED_TYPE" ]; then
            echo -e "${GREEN}✓ Type: $ACTUAL_TYPE${NC}"
        else
            echo -e "${RED}✗ Type mismatch!${NC}"
            echo -e "  Expected: $EXPECTED_TYPE"
            echo -e "  Got: $ACTUAL_TYPE"
            ((FAIL_COUNT++))
            continue
        fi
        
        # Check publishing frequency (sample for 5 seconds)
        echo "  Measuring frequency (5s sample)..."
        FREQ_OUTPUT=$(timeout 5 ros2 topic hz "$topic" 2>/dev/null | grep "average rate:" || echo "")
        
        if [ -n "$FREQ_OUTPUT" ]; then
            ACTUAL_FREQ=$(echo "$FREQ_OUTPUT" | awk '{print $3}')
            EXPECTED_FREQ="${TOPIC_FREQ[$topic]}"
            
            # Allow ±10% tolerance
            MIN_FREQ=$(echo "$EXPECTED_FREQ * 0.9" | bc)
            MAX_FREQ=$(echo "$EXPECTED_FREQ * 1.1" | bc)
            
            if (( $(echo "$ACTUAL_FREQ >= $MIN_FREQ && $ACTUAL_FREQ <= $MAX_FREQ" | bc -l) )); then
                echo -e "${GREEN}✓ Frequency: ${ACTUAL_FREQ} Hz (expected: ${EXPECTED_FREQ} Hz)${NC}"
                ((PASS_COUNT++))
            else
                echo -e "${YELLOW}⚠ Frequency: ${ACTUAL_FREQ} Hz (expected: ${EXPECTED_FREQ} Hz ±10%)${NC}"
                ((PASS_COUNT++))  # Still count as pass, just warning
            fi
        else
            echo -e "${RED}✗ Topic not publishing!${NC}"
            ((FAIL_COUNT++))
        fi
        
    else
        echo -e "${RED}✗ Topic does not exist${NC}"
        ((FAIL_COUNT++))
    fi
    
    echo ""
done

# Summary
echo "=================================================="
echo "Verification Summary"
echo "=================================================="
echo -e "Topics passed:  ${GREEN}$PASS_COUNT${NC}"
echo -e "Topics failed:  ${RED}$FAIL_COUNT${NC}"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed!${NC}"
    echo ""
    echo "You can now visualize topics in RViz2:"
    echo "  rviz2"
    echo "  Add -> Image -> Topic: /left/image_raw"
    exit 0
else
    echo -e "${RED}✗ Some checks failed!${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "1. Verify Isaac Sim is running"
    echo "2. Check ROS 2 bridge extension is enabled in Isaac Sim"
    echo "3. Verify ROS_DOMAIN_ID matches (current: ${ROS_DOMAIN_ID:-0})"
    echo "4. Check for errors in Isaac Sim console"
    exit 1
fi
