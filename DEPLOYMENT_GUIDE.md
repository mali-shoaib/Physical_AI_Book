# Chatbot Deployment Guide

## Problem Overview

Your chatbot currently shows: **"❌ Cannot connect to chatbot service. Is the backend running?"**

This happens because:
1. **Frontend** (Vercel): Trying to connect to `localhost:8000` - which doesn't exist in production
2. **Backend** (Hugging Face): Running a Gradio app instead of the required FastAPI backend

## Solution: Deploy FastAPI Backend

You need to deploy the FastAPI backend from `backend/api.py` to a hosting service.

---

## Option 1: Hugging Face Spaces (Recommended - Free)

### Step 1: Create a New Space

1. Go to https://huggingface.co/spaces
2. Click **"Create new Space"**
3. Settings:
   - **Name**: `physical-ai-textbook-api` (or your choice)
   - **SDK**: Select **Docker** (not Gradio!)
   - **Hardware**: CPU Basic (free tier)
   - **Visibility**: Public

### Step 2: Create Dockerfile

Create `Dockerfile` in your project root:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Copy requirements
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY backend/ ./backend/
COPY specs/ ./specs/

# Expose port
EXPOSE 7860

# Run FastAPI with uvicorn
CMD ["uvicorn", "backend.api:app", "--host", "0.0.0.0", "--port", "7860"]
```

### Step 3: Create Requirements File

Create `backend/requirements.txt`:

```txt
fastapi==0.115.0
uvicorn[standard]==0.32.0
pydantic==2.9.0
python-dotenv==1.0.1
loguru==0.7.2
cohere==5.11.0
qdrant-client==1.11.3
```

### Step 4: Deploy to Hugging Face

```bash
# Initialize git in your backend directory (if not already done)
cd backend
git init
git add .
git commit -m "Initial FastAPI backend"

# Add Hugging Face remote
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-textbook-api
git push hf main
```

### Step 5: Configure Environment Variables in HF Space

1. Go to your Space settings
2. Add these secrets:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant instance URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `OPENAI_API_KEY`: Your OpenAI API key
   - `DOCUSAURUS_BASE_URL`: https://physical-ai-robotics-textbook-xi.vercel.app

### Step 6: Get Your Backend URL

Once deployed, your backend will be at:
```
https://YOUR_USERNAME-physical-ai-textbook-api.hf.space
```

---

## Option 2: Railway.app (Alternative - Free Tier Available)

1. Go to https://railway.app
2. Create new project
3. Deploy from GitHub repo
4. Set environment variables in Railway dashboard
5. Get your deployment URL (e.g., `https://your-app.railway.app`)

---

## Option 3: Render.com (Alternative - Free Tier Available)

1. Go to https://render.com
2. Create new Web Service
3. Connect your GitHub repo
4. Build Command: `pip install -r backend/requirements.txt`
5. Start Command: `uvicorn backend.api:app --host 0.0.0.0 --port $PORT`
6. Add environment variables
7. Get your deployment URL

---

## Configure Frontend (Vercel)

### Method 1: Using Vercel Environment Variables (Recommended)

1. Go to your Vercel project dashboard
2. Navigate to **Settings → Environment Variables**
3. Add new variable:
   - **Name**: `CHATBOT_API_URL`
   - **Value**: Your backend URL (e.g., `https://YOUR_USERNAME-physical-ai-textbook-api.hf.space`)
   - **Environment**: Production, Preview, Development
4. Redeploy your site

### Method 2: Using .env.local (Local Testing)

Create `.env.local` in your project root:

```env
CHATBOT_API_URL=https://YOUR_USERNAME-physical-ai-textbook-api.hf.space
```

---

## Testing the Connection

### 1. Test Backend Health

```bash
curl https://YOUR_BACKEND_URL/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-24T...",
  "version": "1.0.0"
}
```

### 2. Test Chat Endpoint

```bash
curl -X POST https://YOUR_BACKEND_URL/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

### 3. Check Browser Console

1. Open your site: https://physical-ai-robotics-textbook-xi.vercel.app
2. Open DevTools (F12)
3. Check Console tab - you should see:
   ```
   🤖 ChatbotWidget initialized
   📡 Backend API URL: https://YOUR_BACKEND_URL
   🌍 Environment: production
   ```

---

## Troubleshooting

### Error: "Cannot connect to chatbot service"

**Check:**
1. Backend is running (test `/api/health` endpoint)
2. CORS is configured correctly (backend allows your Vercel domain)
3. Environment variable is set in Vercel
4. Redeploy after setting environment variables

### Error: "CORS policy blocked"

**Fix:**
The backend `api.py` already includes your Vercel domain in CORS config. If you have a custom domain, add it to `ALLOWED_ORIGINS` environment variable:

```
ALLOWED_ORIGINS=https://custom-domain.com,https://another-domain.com
```

### Backend Returns 404

**Check:**
- Endpoint is `/api/chat/query` (not `/chat` or other)
- Using POST method (not GET)
- Content-Type header is `application/json`

---

## Quick Start Commands

### Local Development

```bash
# Terminal 1: Start backend
cd backend
python -m uvicorn api:app --reload --port 8000

# Terminal 2: Start frontend
npm run start
```

### Production URLs

- **Frontend**: https://physical-ai-robotics-textbook-xi.vercel.app
- **Backend**: Set this after deployment (see above)

---

## Summary

1. ✅ Frontend updated to use environment variables
2. ✅ Backend CORS configured for Vercel domain
3. ⏳ **Next Step**: Deploy FastAPI backend to Hugging Face/Railway/Render
4. ⏳ **Then**: Configure `CHATBOT_API_URL` in Vercel
5. ⏳ **Finally**: Redeploy and test!

---

## Need Help?

- Backend deployment issues: Check backend logs in your hosting platform
- Frontend issues: Check browser console (F12 → Console tab)
- CORS errors: Verify your domain is in `allowed_origins` in `backend/api.py:168`
