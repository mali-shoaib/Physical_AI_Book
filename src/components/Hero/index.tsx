import React from 'react';
import CTAButton from '@site/src/components/CTAButton';
import styles from './styles.module.css';

export interface HeroProps {
  /** Main heading text (48px desktop, 32px mobile) */
  title: string;

  /** Subheading text (28px desktop, 20px mobile) */
  subtitle: string;

  /** CTA button text */
  ctaText: string;

  /** CTA button link (internal route like /docs/intro) */
  ctaLink: string;

  /** Optional hero image URL */
  imageUrl?: string;

  /** Optional hero image alt text */
  imageAlt?: string;
}

export default function Hero({
  title,
  subtitle,
  ctaText,
  ctaLink,
  imageUrl,
  imageAlt,
}: HeroProps): JSX.Element {
  return (
    <section className={styles.hero} aria-labelledby="hero-title">
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <h1 id="hero-title" className={styles.title}>
            {title}
          </h1>
          <h2 className={styles.subtitle}>
            {subtitle}
          </h2>
          <CTAButton text={ctaText} href={ctaLink} />
        </div>
        {imageUrl && (
          <div className={styles.heroImage}>
            <img src={imageUrl} alt={imageAlt || title} />
          </div>
        )}
      </div>
    </section>
  );
}
