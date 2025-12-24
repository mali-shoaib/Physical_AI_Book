/**
 * TypeScript type definitions for RAG Chatbot Widget
 */

export interface Citation {
  module_name: string;
  chapter_id: string;
  source_url: string;
  similarity_score: number;
}

export interface ChatMessage {
  role: "user" | "assistant";
  content: string;
  timestamp: Date;
  citations?: Citation[];
}

export interface ChatRequest {
  query: string;
  conversation_id?: string | null;
  context?: string | null;
}

export interface ResponseMetadata {
  retrieval_count: number;
  tokens_used: number;
  grounded: boolean;
}

export interface ChatResponse {
  answer: string;
  citations: Citation[];
  conversation_id: string;
  metadata: ResponseMetadata;
}

export interface ChatWidgetState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  conversationId: string | null;
}
