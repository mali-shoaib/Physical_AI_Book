import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export interface CTAButtonProps {
  /** Button text */
  text: string;

  /** Link destination (internal route or external URL) */
  href: string;

  /** Visual style variant (default: 'primary') */
  variant?: 'primary' | 'secondary';
}

export default function CTAButton({
  text,
  href,
  variant = 'primary',
}: CTAButtonProps): JSX.Element {
  const buttonClass = variant === 'primary'
    ? styles.ctaButton
    : styles.ctaButtonSecondary;

  return (
    <Link
      to={href}
      className={buttonClass}
      role="button"
      aria-label={`${text} - Navigate to ${href}`}>
      {text}
    </Link>
  );
}
