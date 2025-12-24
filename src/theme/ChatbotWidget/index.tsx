import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import type { ChatMessage, ChatRequest, ChatResponse, ChatWidgetState } from './types';
import styles from './styles.module.css';

export default function ChatbotWidget(): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.chatbotApiUrl as string) || 'https://shoaibali-s-srag.hf.space';
  const [mounted, setMounted] = useState(false);
  const [state, setState] = useState<ChatWidgetState>({
    messages: [],
    isOpen: false,
    isLoading: false,
    error: null,
    conversationId: null,
  });

  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Client-side only rendering
  useEffect(() => {
    setMounted(true);
    console.log('🤖 ChatbotWidget initialized');
    console.log('📡 Backend API URL:', API_BASE_URL);
    console.log('🌍 Environment:', process.env.NODE_ENV);
    console.log('🔧 CustomFields:', siteConfig.customFields);

    // Alert user if using localhost (development mode)
    if (API_BASE_URL.includes('localhost')) {
      console.warn('⚠️ WARNING: Using localhost backend URL. This will not work in production!');
    }
  }, []);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    if (mounted) {
      messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  }, [state.messages, mounted]);

  // Load conversation ID from session storage
  useEffect(() => {
    if (mounted) {
      const savedConvId = sessionStorage.getItem('chatbot_conversation_id');
      if (savedConvId) {
        setState((prev) => ({ ...prev, conversationId: savedConvId }));
      }
    }
  }, [mounted]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || state.isLoading) return;

    const userMessage: ChatMessage = {
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setState((prev) => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      isLoading: true,
      error: null,
    }));

    setInputValue('');

    try {
      const requestPayload: ChatRequest = {
        query: inputValue,
        conversation_id: state.conversationId,
      };

      console.log('🚀 Sending request to:', `${API_BASE_URL}/api/chat/query`);
      console.log('📦 Payload:', requestPayload);

      const response = await axios.post<ChatResponse>(
        `${API_BASE_URL}/api/chat/query`,
        requestPayload,
        { timeout: 30000 }
      );

      console.log('✅ Response received:', response.data);

      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: response.data.answer,
        timestamp: new Date(),
        citations: response.data.citations,
      };

      setState((prev) => ({
        ...prev,
        messages: [...prev.messages, assistantMessage],
        isLoading: false,
        conversationId: response.data.conversation_id,
      }));

      // Save conversation ID
      sessionStorage.setItem('chatbot_conversation_id', response.data.conversation_id);
    } catch (error) {
      console.error('❌ Chat request failed:', error);

      let errorMessage = 'Failed to get response. Please try again.';

      if (axios.isAxiosError(error)) {
        console.error('📍 Request URL was:', error.config?.url);
        console.error('📍 Error details:', {
          status: error.response?.status,
          statusText: error.response?.statusText,
          data: error.response?.data
        });

        if (error.response) {
          errorMessage = error.response.data?.detail || `Error ${error.response.status}: ${error.response.statusText}`;
        } else if (error.request) {
          errorMessage = 'Cannot connect to chatbot service. Backend URL: ' + API_BASE_URL;
        }
      }

      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
    }
  };

  const toggleWidget = () => {
    setState((prev) => ({ ...prev, isOpen: !prev.isOpen }));
  };

  const handleReset = async () => {
    // Show confirmation dialog
    const confirmed = window.confirm(
      'Are you sure you want to reset the conversation? This will clear all messages and start fresh.'
    );

    if (!confirmed) return;

    try {
      // Call reset endpoint if conversation exists
      if (state.conversationId) {
        await axios.post(`${API_BASE_URL}/api/chat/reset/${state.conversationId}`);
      }

      // Clear local state
      setState((prev) => ({
        ...prev,
        messages: [],
        conversationId: null,
        error: null,
      }));

      // Clear session storage
      sessionStorage.removeItem('chatbot_conversation_id');

      console.log('🔄 Conversation reset successfully');
    } catch (error) {
      console.error('Failed to reset conversation:', error);
      // Still clear local state even if API call fails
      setState((prev) => ({
        ...prev,
        messages: [],
        conversationId: null,
        error: null,
      }));
      sessionStorage.removeItem('chatbot_conversation_id');
    }
  };

  // Don't render until client-side
  if (!mounted) {
    return null;
  }

  return (
    <div className={styles.chatbotContainer}>
      {/* Toggle Button */}
      <button
        className={styles.toggleButton}
        onClick={toggleWidget}
        aria-label="Toggle chatbot"
      >
        {state.isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Widget */}
      {state.isOpen && (
        <div className={styles.chatWidget}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerLeft}>
              <h3>Textbook Assistant</h3>
              <span className={styles.statusDot}></span>
            </div>
            {state.messages.length > 0 && (
              <button
                className={styles.resetButton}
                onClick={handleReset}
                title="Reset conversation"
                aria-label="Reset conversation"
              >
                🔄
              </button>
            )}
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {state.messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Ask me anything about the Physical AI textbook!</p>
              </div>
            )}

            {state.messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${
                  msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                <div className={styles.messageContent}>{msg.content}</div>

                {/* Citations */}
                {msg.citations && msg.citations.length > 0 && (
                  <div className={styles.citations}>
                    <strong>Sources:</strong>
                    {msg.citations.map((cit, citIdx) => (
                      <div key={citIdx} className={styles.citation}>
                        <a href={cit.source_url} target="_blank" rel="noopener noreferrer">
                          {cit.module_name} - {cit.chapter_id}
                        </a>
                        <span className={styles.score}>
                          ({(cit.similarity_score * 100).toFixed(0)}%)
                        </span>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {state.isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.loadingSpinner}>
                  <span>Thinking...</span>
                </div>
              </div>
            )}

            {state.error && (
              <div className={styles.errorMessage}>
                ❌ {state.error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question..."
              className={styles.input}
              disabled={state.isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={state.isLoading || !inputValue.trim()}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
}
