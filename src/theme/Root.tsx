import React, { useEffect } from 'react';
import ChatbotWidget from './ChatbotWidget';

// Docusaurus Root component wrapper
// This component wraps the entire app and is rendered on every page
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  useEffect(() => {
    console.log('🔵 Root.tsx LOADED - Chatbot should initialize');
  }, []);

  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
