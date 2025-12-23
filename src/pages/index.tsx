import React from 'react';
import Layout from '@theme/Layout';
import Hero from '@site/src/components/Hero';

export default function LandingPage(): JSX.Element {
  return (
    <Layout
      title="Educational Textbook Platform"
      description="Open-source textbooks designed for the modern learner. High-quality educational materials, freely accessible to all.">
      <main>
        <Hero
          title="Physical AI & Humanoid Robotics"
          subtitle="A comprehensive textbook for robotics students and developers"
          ctaText="Start Learning"
          ctaLink="/books/docs/intro"
          imageUrl="https://images.unsplash.com/photo-1485827404703-89b55fcc595e?w=800&auto=format&fit=crop&q=80"
          imageAlt="Humanoid robot with advanced AI capabilities"
        />
      </main>
    </Layout>
  );
}
