import React from 'react';
import ChatbotWidget from './ChatbotWidget';

// Docusaurus Root component wrapper
// This component wraps the entire app and is rendered on every page
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
