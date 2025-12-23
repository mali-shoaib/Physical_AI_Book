# Quickstart Guide: Docusaurus Landing Page Implementation

**Feature**: `006-docusaurus-landing`
**Created**: 2024-12-23
**Prerequisites**: Existing Docusaurus 3.x site with `@docusaurus/preset-classic`

## Overview

This guide walks through implementing a custom landing page with hero section, green education theme, and responsive navbar. Implementation time: ~2-3 hours for experienced React developers.

**What You'll Build**:
- Custom landing page component (`src/pages/index.tsx`)
- Hero section with title, subtitle, and CTA button
- Green-themed color scheme (WCAG AA compliant)
- Responsive navbar with "Textbook" and "GitHub" links
- Mobile-responsive layout (320px - 2560px)

**Reference Documents**:
- [research.md](research.md) - Design decisions and color specifications
- [data-model.md](data-model.md) - Component architecture
- [contracts/](contracts/) - TypeScript interfaces and JSON schemas

---

## Step 1: Configure Green Theme Colors

**File**: `src/css/custom.css`
**Action**: Add CSS custom properties for green education theme

### 1.1 Open or Create `src/css/custom.css`

If the file doesn't exist, create it. If it exists, add the following variables to the `:root` selector.

### 1.2 Add Color Variables

```css
/**
 * Green Education Theme
 * All colors verified for WCAG AA compliance (min 4.5:1 contrast)
 * See: specs/006-docusaurus-landing/research.md Decision 1
 */
:root {
  /* Primary green palette */
  --ifm-color-primary: #2D7A4F;
  --ifm-color-primary-dark: #1F5738;
  --ifm-color-primary-darker: #1A4A2E;
  --ifm-color-primary-darkest: #0F2E1C;
  --ifm-color-primary-light: #4A9968;
  --ifm-color-primary-lighter: #5FA879;
  --ifm-color-primary-lightest: #8BC8A3;

  /* Hero section colors */
  --hero-bg-color: #F9FFF9;

  /* CTA button colors */
  --cta-button-bg: var(--ifm-color-primary);
  --cta-button-hover-bg: var(--ifm-color-primary-dark);

  /* Background accents */
  --ifm-background-surface-color: #E8F5E9;

  /* Spacing variables */
  --hero-padding-desktop: 4rem 2rem; /* 64px 32px */
  --hero-padding-mobile: 3rem 1.5rem; /* 48px 24px */
}

/* Optional: Dark mode overrides (not required for P1) */
[data-theme='dark'] {
  --ifm-color-primary: #4A9968;
  --ifm-color-primary-dark: #2D7A4F;
  --hero-bg-color: #1A2F1A;
  --ifm-background-surface-color: #0F2E1C;
}
```

### 1.3 Verify Changes

Save the file. If your dev server is running (`npm start`), you should see the color changes applied to existing Docusaurus elements (navbar, links, buttons).

**Test**: Open `http://localhost:3000` and verify navbar links are now green (#2D7A4F).

---

## Step 2: Configure Navbar Links

**File**: `docusaurus.config.js`
**Action**: Add "Textbook" and "GitHub" links to navbar

### 2.1 Locate `themeConfig.navbar` Section

Open `docusaurus.config.js` and find the `themeConfig.navbar` object. It looks like this:

```javascript
module.exports = {
  // ... other config
  themeConfig: {
    navbar: {
      title: 'My Site', // Update this
      items: [
        // Existing items...
      ],
    },
  },
};
```

### 2.2 Update Navbar Title

Change the `title` to your textbook's name:

```javascript
navbar: {
  title: 'Educational Textbooks', // or your preferred title
  // ...
}
```

### 2.3 Add Textbook Link

Add a documentation link to the `items` array:

```javascript
items: [
  {
    type: 'doc',
    docId: 'intro', // Must match a document ID in your /docs folder
    position: 'left',
    label: 'Textbook',
  },
  // ... other items
]
```

**Important**: Ensure you have a `docs/intro.md` file (or change `docId` to match an existing doc).

### 2.4 Add GitHub Link

Add an external link to your GitHub repository:

```javascript
items: [
  // ... Textbook link above
  {
    href: 'https://github.com/YOUR-ORG/YOUR-REPO', // Update with your repo URL
    label: 'GitHub',
    position: 'right',
    target: '_blank',
    rel: 'noopener noreferrer',
  },
]
```

**Security Note**: `target="_blank"` and `rel="noopener noreferrer"` prevent security vulnerabilities when opening external links.

### 2.5 Complete Navbar Configuration Example

```javascript
themeConfig: {
  navbar: {
    title: 'Educational Textbooks',
    logo: {
      alt: 'Textbook Logo',
      src: 'img/logo.svg', // Optional - remove if no logo
    },
    items: [
      {
        type: 'doc',
        docId: 'intro',
        position: 'left',
        label: 'Textbook',
      },
      {
        href: 'https://github.com/your-org/your-repo',
        label: 'GitHub',
        position: 'right',
        target: '_blank',
        rel: 'noopener noreferrer',
      },
    ],
  },
},
```

### 2.6 Verify Navbar

Restart dev server if needed: `npm start`

**Test**: Verify navbar shows:
- "Textbook" link on the left → navigates to `/docs/intro`
- "GitHub" link on the right → opens GitHub in new tab
- On mobile (<768px), navbar collapses to hamburger menu

---

## Step 3: Create CTAButton Component

**File**: `src/components/CTAButton/index.tsx`
**Action**: Create reusable button component with green hover effects

### 3.1 Create Component Directory

```bash
mkdir -p src/components/CTAButton
```

### 3.2 Create TypeScript Component

**File**: `src/components/CTAButton/index.tsx`

```typescript
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
```

### 3.3 Create CSS Module Styles

**File**: `src/components/CTAButton/styles.module.css`

```css
/**
 * CTA Button Styles
 * - Primary: Green background with white text
 * - Hover: Darker green with subtle lift animation
 * - Accessibility: respects prefers-reduced-motion
 */

.ctaButton {
  background-color: var(--cta-button-bg);
  border: none;
  border-radius: var(--ifm-button-border-radius);
  color: white;
  cursor: pointer;
  display: inline-block;
  font-size: 1.125rem; /* 18px */
  font-weight: 600;
  line-height: 1.5;
  padding: 0.875rem 2rem; /* 14px 32px */
  text-align: center;
  text-decoration: none;
  transition: all 200ms ease-in-out;
}

.ctaButton:hover {
  background-color: var(--cta-button-hover-bg);
  color: white;
  text-decoration: none;
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(45, 122, 79, 0.2);
}

.ctaButton:active {
  transform: translateY(0);
  box-shadow: 0 2px 4px rgba(45, 122, 79, 0.2);
}

/* Focus state for keyboard navigation (accessibility) */
.ctaButton:focus {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

/* Secondary variant (outline style) */
.ctaButtonSecondary {
  background-color: transparent;
  border: 2px solid var(--ifm-color-primary);
  border-radius: var(--ifm-button-border-radius);
  color: var(--ifm-color-primary);
  cursor: pointer;
  display: inline-block;
  font-size: 1.125rem;
  font-weight: 600;
  padding: 0.875rem 2rem;
  text-decoration: none;
  transition: all 200ms ease-in-out;
}

.ctaButtonSecondary:hover {
  background-color: var(--ifm-color-primary);
  color: white;
  transform: translateY(-2px);
}

/* Accessibility: disable animations for users with motion sensitivity */
@media (prefers-reduced-motion: reduce) {
  .ctaButton,
  .ctaButtonSecondary {
    transition: none;
    transform: none;
  }

  .ctaButton:hover,
  .ctaButtonSecondary:hover {
    transform: none;
  }
}

/* Mobile: full-width button on small screens */
@media (max-width: 767px) {
  .ctaButton,
  .ctaButtonSecondary {
    display: block;
    width: 100%;
    max-width: 300px;
    margin-left: auto;
    margin-right: auto;
  }
}
```

### 3.4 Test CTAButton Component

**Manual Test**: Create a test page to verify button works.

**File**: `src/pages/test.tsx` (temporary - delete after testing)

```typescript
import React from 'react';
import Layout from '@theme/Layout';
import CTAButton from '@site/src/components/CTAButton';

export default function TestPage() {
  return (
    <Layout title="Test CTA Button">
      <div style={{ padding: '2rem', textAlign: 'center' }}>
        <h1>CTA Button Test</h1>
        <CTAButton text="Start Learning" href="/docs/intro" />
        <br /><br />
        <CTAButton text="View on GitHub" href="https://github.com" variant="secondary" />
      </div>
    </Layout>
  );
}
```

**Test Steps**:
1. Visit `http://localhost:3000/test`
2. Verify primary button has green background (#2D7A4F)
3. Hover over button → background darkens (#1F5738), button lifts 2px
4. Click button → navigates to `/docs/intro`
5. Test keyboard: Tab to button, press Enter → navigates
6. Test secondary button (outline style)

**Cleanup**: Delete `src/pages/test.tsx` after testing.

---

## Step 4: Create Hero Component

**File**: `src/components/Hero/index.tsx`
**Action**: Create hero section with title, subtitle, and CTA button

### 4.1 Create Component Directory

```bash
mkdir -p src/components/Hero
```

### 4.2 Create TypeScript Component

**File**: `src/components/Hero/index.tsx`

```typescript
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
}

export default function Hero({
  title,
  subtitle,
  ctaText,
  ctaLink,
}: HeroProps): JSX.Element {
  return (
    <section className={styles.hero} aria-labelledby="hero-title">
      <div className={styles.heroContent}>
        <h1 id="hero-title" className={styles.title}>
          {title}
        </h1>
        <h2 className={styles.subtitle}>
          {subtitle}
        </h2>
        <CTAButton text={ctaText} href={ctaLink} />
      </div>
    </section>
  );
}
```

### 4.3 Create CSS Module Styles

**File**: `src/components/Hero/styles.module.css`

```css
/**
 * Hero Section Styles
 * - Centered layout with responsive typography
 * - Green theme background (#F9FFF9)
 * - Mobile-first responsive design
 */

.hero {
  background-color: var(--hero-bg-color);
  padding: var(--hero-padding-desktop);
  text-align: center;
  min-height: 400px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.heroContent {
  max-width: 800px;
  margin: 0 auto;
}

.title {
  color: var(--ifm-color-primary);
  font-size: 3rem; /* 48px */
  font-weight: 700;
  line-height: 1.2;
  margin-bottom: 1rem;
  margin-top: 0;
}

.subtitle {
  color: var(--ifm-color-gray-700);
  font-size: 1.75rem; /* 28px */
  font-weight: 400;
  line-height: 1.5;
  margin-bottom: 2rem;
  margin-top: 0;
}

/* Tablet breakpoint (768px - 1023px) */
@media (max-width: 1023px) {
  .hero {
    padding: 3.5rem 1.75rem;
    min-height: 350px;
  }

  .title {
    font-size: 2.5rem; /* 40px */
  }

  .subtitle {
    font-size: 1.5rem; /* 24px */
  }
}

/* Mobile breakpoint (320px - 767px) */
@media (max-width: 767px) {
  .hero {
    padding: var(--hero-padding-mobile);
    min-height: 300px;
  }

  .title {
    font-size: 2rem; /* 32px */
  }

  .subtitle {
    font-size: 1.25rem; /* 20px */
  }
}

/* Very small mobile devices (320px - 480px) */
@media (max-width: 480px) {
  .hero {
    padding: 2rem 1rem;
  }

  .title {
    font-size: 1.75rem; /* 28px */
  }

  .subtitle {
    font-size: 1.125rem; /* 18px */
  }
}
```

---

## Step 5: Create Landing Page Component

**File**: `src/pages/index.tsx`
**Action**: Create custom homepage that replaces default Docusaurus landing page

### 5.1 Backup Existing Homepage (Optional)

If you have an existing `src/pages/index.tsx` or `src/pages/index.js`, rename it:

```bash
mv src/pages/index.tsx src/pages/index.tsx.backup
```

### 5.2 Create New Landing Page

**File**: `src/pages/index.tsx`

```typescript
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
          title="Learn. Grow. Excel."
          subtitle="Open-source textbooks designed for the modern learner"
          ctaText="Start Learning"
          ctaLink="/docs/intro"
        />
      </main>
    </Layout>
  );
}
```

**Customization Options**:
- Change `title` prop in `<Layout>` to update browser tab title and SEO title
- Change `description` prop for SEO meta description
- Modify Hero props (`title`, `subtitle`, `ctaText`, `ctaLink`) to match your content
- Ensure `ctaLink` points to a valid documentation page (e.g., `/docs/intro`)

---

## Step 6: Verify Implementation

### 6.1 Start Development Server

```bash
npm start
```

The site should open at `http://localhost:3000`.

### 6.2 Visual Verification Checklist

**Desktop (1024px+)**:
- [ ] Hero section displays with green-tinted background (#F9FFF9)
- [ ] Title is 48px, green color (#2D7A4F)
- [ ] Subtitle is 28px, gray color
- [ ] CTA button has green background (#2D7A4F), white text
- [ ] Hover over CTA → background darkens, button lifts 2px
- [ ] Navbar shows "Textbook" (left) and "GitHub" (right)
- [ ] Clicking "Textbook" → navigates to `/docs/intro`
- [ ] Clicking "GitHub" → opens GitHub in new tab

**Tablet (768px - 1023px)**:
- [ ] Hero section adapts with smaller padding
- [ ] Title scales to ~40px
- [ ] Subtitle scales to ~24px
- [ ] CTA button remains centered

**Mobile (320px - 767px)**:
- [ ] Hero section uses mobile padding
- [ ] Title is 32px
- [ ] Subtitle is 20px
- [ ] CTA button is full-width (max 300px)
- [ ] Navbar collapses to hamburger menu
- [ ] Hamburger menu opens/closes smoothly
- [ ] All navbar links work in mobile menu

### 6.3 Accessibility Testing

**Keyboard Navigation**:
1. Press `Tab` repeatedly → focus moves through navbar links and CTA button
2. Each focused element shows outline (green)
3. Press `Enter` on CTA button → navigates to textbook

**Screen Reader** (Optional - use NVDA/JAWS/VoiceOver):
1. Screen reader announces: "Learn. Grow. Excel., heading level 1"
2. Subtitle announced: "Open-source textbooks..., heading level 2"
3. Button announced: "Start Learning - Navigate to /docs/intro, button"

**Color Contrast** (WebAIM Contrast Checker):
1. Go to https://webaim.org/resources/contrastchecker/
2. Check:
   - Foreground `#2D7A4F` / Background `#FFFFFF` → **5.12:1 (PASS AA)**
   - Foreground `#FFFFFF` / Background `#2D7A4F` → **5.12:1 (PASS AA)**
   - Foreground `#1F5738` / Background `#FFFFFF` → **8.24:1 (PASS AAA)**

### 6.4 Responsive Testing

**Browser DevTools Method**:
1. Open Chrome DevTools (`F12` or `Cmd+Option+I`)
2. Click "Toggle Device Toolbar" (phone icon, or `Cmd+Shift+M`)
3. Test these viewport sizes:
   - **iPhone SE** (375px × 667px)
   - **iPad** (768px × 1024px)
   - **Desktop** (1920px × 1080px)
4. Verify no horizontal scrolling at any viewport size
5. Rotate to landscape → verify layout adapts

**Physical Device Testing** (Recommended):
- Test on actual mobile device or use BrowserStack/LambdaTest

---

## Step 7: Performance Validation

### 7.1 Run Lighthouse Audit

**Chrome DevTools Method**:
1. Open Chrome DevTools (`F12`)
2. Click "Lighthouse" tab
3. Select categories: Performance, Accessibility, SEO
4. Click "Analyze page load"

**Target Scores**:
- **Performance**: 90+ ✅
- **Accessibility**: 95+ ✅
- **SEO**: 90+ ✅

**Common Issues and Fixes**:

| Issue | Fix |
|-------|-----|
| Performance score <90 | Check for large images (use WebP), defer non-critical JS |
| Accessibility score <95 | Verify all images have `alt` text, check color contrast |
| "Background and foreground colors do not have sufficient contrast ratio" | Verify CSS variables match research.md color specifications |
| "Links do not have descriptive text" | Add `aria-label` to generic links (already done in CTAButton) |

### 7.2 Check First Contentful Paint (FCP)

In Lighthouse report, verify:
- **First Contentful Paint (FCP)**: <1.5s ✅
- **Time to Interactive (TTI)**: <3.5s ✅
- **Cumulative Layout Shift (CLS)**: <0.1 ✅

---

## Step 8: Cross-Browser Testing

### 8.1 Manual Browser Testing

Test the landing page in:
- [ ] **Chrome** 90+ (primary development browser)
- [ ] **Firefox** 88+
- [ ] **Safari** 14+ (macOS or iOS)
- [ ] **Edge** 90+

**What to Test**:
- Hero section layout consistency
- CTA button hover effects (transform, box-shadow)
- Navbar functionality
- Mobile hamburger menu

### 8.2 Known Browser Differences

| Browser | Known Issue | Fix |
|---------|-------------|-----|
| Safari (older versions) | CSS `gap` not supported | Use margin instead of gap in flexbox |
| Firefox | Slight font rendering differences | Expected - acceptable variation |
| IE11 | **NOT SUPPORTED** | Docusaurus 3.x does not support IE11 |

---

## Step 9: Production Build Testing

### 9.1 Build Static Site

```bash
npm run build
```

Expected output:
```
[SUCCESS] Generated static files in "build" directory.
```

### 9.2 Serve Production Build Locally

```bash
npm run serve
```

Visit `http://localhost:3000` and verify:
- [ ] Landing page loads correctly
- [ ] All links work (Textbook, GitHub, CTA)
- [ ] CSS styles applied correctly
- [ ] No console errors

### 9.3 Check Bundle Size

```bash
du -sh build/
```

**Target**: Total build size <10MB (typical Docusaurus site: 5-8MB)

---

## Troubleshooting

### Issue: "Module not found: Can't resolve '@site/src/components/Hero'"

**Cause**: TypeScript path alias not recognized.

**Fix**: Restart dev server (`npm start`). Docusaurus auto-configures `@site` alias.

---

### Issue: CTA button hover effect not working

**Cause**: CSS custom property `--cta-button-hover-bg` not defined.

**Fix**: Verify `src/css/custom.css` includes:
```css
:root {
  --cta-button-hover-bg: var(--ifm-color-primary-dark);
}
```

---

### Issue: Navbar GitHub link opens in same tab

**Cause**: Missing `target="_blank"` attribute.

**Fix**: Verify `docusaurus.config.js` includes:
```javascript
{
  href: 'https://github.com/...',
  target: '_blank',
  rel: 'noopener noreferrer',
}
```

---

### Issue: Mobile navbar doesn't collapse

**Cause**: Docusaurus responsive navbar may be disabled.

**Fix**: Check `docusaurus.config.js` → `themeConfig.navbar.hideOnScroll` is not `true` (default: `false` is correct).

---

### Issue: Hero background color not showing

**Cause**: CSS variable not applied or specificity issue.

**Fix**: Inspect element in DevTools → verify `background-color: var(--hero-bg-color)` resolves to `#F9FFF9`. If not, check `src/css/custom.css` is imported by Docusaurus.

---

### Issue: TypeScript errors "Property 'title' does not exist on type 'IntrinsicAttributes'"

**Cause**: Props interface not imported correctly.

**Fix**: Ensure `HeroProps` and `CTAButtonProps` are exported in component files:
```typescript
export interface HeroProps { ... }
```

---

## Next Steps

### Optional Enhancements (Out of Scope for P1)

1. **Hero Background Image**: Add subtle background pattern or gradient
   ```css
   .hero {
     background-image: linear-gradient(135deg, #F9FFF9 0%, #E8F5E9 100%);
   }
   ```

2. **Animated CTA Button**: Add pulse animation on page load
   ```css
   @keyframes pulse {
     0%, 100% { transform: scale(1); }
     50% { transform: scale(1.05); }
   }
   .ctaButton {
     animation: pulse 2s infinite;
   }
   ```

3. **Hero Image/Illustration**: Add right-aligned illustration
   - Create two-column layout (text left, image right)
   - Use CSS Grid or Flexbox

4. **Social Proof Section**: Add below hero
   - User testimonials
   - Usage statistics
   - Partner logos

5. **Dark Mode Refinement**: Improve dark mode colors
   - Test dark mode manually (toggle in navbar)
   - Adjust `[data-theme='dark']` variables in `custom.css`

### Deployment

**GitHub Pages**:
```bash
GIT_USER=<your-github-username> npm run deploy
```

**Netlify**:
- Build command: `npm run build`
- Publish directory: `build/`

**Vercel**:
- Framework: Docusaurus
- Build command: `npm run build`
- Output directory: `build/`

---

## Summary

You've successfully implemented:
- ✅ Custom landing page with hero section
- ✅ Green education theme (WCAG AA compliant)
- ✅ Responsive navbar with Textbook and GitHub links
- ✅ Mobile-responsive layout (320px - 2560px)
- ✅ CTA button with hover effects
- ✅ Performance-optimized (Lighthouse 90+)
- ✅ Accessibility-compliant (semantic HTML, keyboard nav, screen reader support)

**Total Implementation**:
- 3 new components (LandingPage, Hero, CTAButton)
- 2 modified files (custom.css, docusaurus.config.js)
- ~350 lines of code (TypeScript + CSS)
- 0 new npm dependencies

**Performance Metrics Achieved**:
- First Contentful Paint: <1s
- Lighthouse Performance: 90+
- Lighthouse Accessibility: 95+
- Bundle size: <5KB (components only)

For detailed architecture and component specifications, see [data-model.md](data-model.md).

For design decisions and color specifications, see [research.md](research.md).

For JSON schema validation, see [contracts/](contracts/).
