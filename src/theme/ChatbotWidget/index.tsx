import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import type { ChatMessage, ChatRequest, ChatResponse, ChatWidgetState } from './types';
import styles from './styles.module.css';

const API_BASE_URL = 'http://localhost:8000';

export default function ChatbotWidget(): JSX.Element {
  const [state, setState] = useState<ChatWidgetState>({
    messages: [],
    isOpen: false,
    isLoading: false,
    error: null,
    conversationId: null,
  });

  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [state.messages]);

  // Load conversation ID from session storage
  useEffect(() => {
    const savedConvId = sessionStorage.getItem('chatbot_conversation_id');
    if (savedConvId) {
      setState((prev) => ({ ...prev, conversationId: savedConvId }));
    }
  }, []);

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

      const response = await axios.post<ChatResponse>(
        `${API_BASE_URL}/api/chat/query`,
        requestPayload,
        { timeout: 30000 }
      );

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
      let errorMessage = 'Failed to get response. Please try again.';

      if (axios.isAxiosError(error)) {
        if (error.response) {
          errorMessage = error.response.data?.detail || errorMessage;
        } else if (error.request) {
          errorMessage = 'Cannot connect to chatbot service. Is the backend running?';
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
            <h3>Textbook Assistant</h3>
            <span className={styles.statusDot}></span>
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
