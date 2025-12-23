# Data Model: Docusaurus Landing Page UI

**Feature**: `006-docusaurus-landing`
**Created**: 2024-12-23
**Input**: Research decisions from [research.md](research.md)

## Component Architecture

### Component Hierarchy

```
LandingPage (src/pages/index.tsx)
└── Layout (@docusaurus/theme-classic)
    └── Hero (src/components/Hero/index.tsx)
        └── CTAButton (src/components/CTAButton/index.tsx)
```

### Component Ownership

- **Docusaurus Core**: Provides `Layout`, `Link`, and responsive utilities (no customization needed)
- **Custom Components**: LandingPage, Hero, CTAButton (feature ownership)
- **Theme System**: Configured via `docusaurus.config.js` and `src/css/custom.css`

## Component Specifications

### 1. LandingPage Component

**File**: `src/pages/index.tsx`
**Type**: Page Component (Docusaurus convention)
**Responsibility**: Root landing page that renders hero section within Docusaurus layout

#### Interface

```typescript
// No props - page components are automatically routed by Docusaurus
export default function LandingPage(): JSX.Element;
```

#### State Management

- **State**: None (stateless presentation component)
- **Side Effects**: None
- **Data Flow**: Passes hardcoded content to Hero component

#### Internal Structure

```typescript
import Layout from '@theme/Layout';
import Hero from '@site/src/components/Hero';

export default function LandingPage() {
  return (
    <Layout
      title="Educational Textbook Platform"
      description="Open-source textbooks for modern education">
      <Hero
        title="Learn. Grow. Excel."
        subtitle="Open-source textbooks designed for the modern learner"
        ctaText="Start Learning"
        ctaLink="/docs/intro"
      />
    </Layout>
  );
}
```

**Design Rationale**:
- Uses Docusaurus `Layout` component for navbar/footer consistency
- Content is hardcoded (no CMS integration per spec scope)
- Title/description for SEO meta tags

---

### 2. Hero Component

**File**: `src/components/Hero/index.tsx`
**Type**: Presentation Component
**Responsibility**: Renders hero section with title, subtitle, and CTA button

#### Interface

```typescript
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

export default function Hero(props: HeroProps): JSX.Element;
```

#### State Management

- **State**: None (stateless presentation component)
- **Props**: Receives content from parent LandingPage
- **Children**: Renders CTAButton with ctaText and ctaLink props

#### Styling Contract

**CSS Module**: `src/components/Hero/styles.module.css`

```css
/* Component uses CSS custom properties from theme */
.hero {
  background-color: var(--hero-bg-color);
  padding: var(--hero-padding-desktop);
  text-align: center;
}

.title {
  color: var(--ifm-color-primary);
  font-size: 3rem; /* 48px */
  font-weight: 700;
  margin-bottom: 1rem;
}

.subtitle {
  color: var(--ifm-color-gray-700);
  font-size: 1.75rem; /* 28px */
  font-weight: 400;
  margin-bottom: 2rem;
}

/* Responsive overrides */
@media (max-width: 767px) {
  .hero { padding: var(--hero-padding-mobile); }
  .title { font-size: 2rem; /* 32px */ }
  .subtitle { font-size: 1.25rem; /* 20px */ }
}
```

**Design Rationale**:
- Uses semantic HTML (`<section>`, `<h1>`, `<h2>`)
- Inherits responsive behavior from Docusaurus Infima framework
- CSS variables allow theme customization without component changes

---

### 3. CTAButton Component

**File**: `src/components/CTAButton/index.tsx`
**Type**: Presentation Component
**Responsibility**: Renders styled button with hover effects using Docusaurus Link

#### Interface

```typescript
export interface CTAButtonProps {
  /** Button text */
  text: string;

  /** Link destination (internal route or external URL) */
  href: string;

  /** Visual style variant (default: 'primary') */
  variant?: 'primary' | 'secondary';
}

export default function CTAButton(props: CTAButtonProps): JSX.Element;
```

#### State Management

- **State**: Hover state (managed via CSS `:hover` pseudo-class, no React state)
- **Props**: Receives text, href, and optional variant from parent
- **Interaction**: Uses Docusaurus `Link` for client-side navigation

#### Styling Contract

**CSS Module**: `src/components/CTAButton/styles.module.css`

```css
.ctaButton {
  background-color: var(--cta-button-bg);
  border-radius: var(--ifm-button-border-radius);
  color: white;
  display: inline-block;
  font-size: 1.125rem; /* 18px */
  font-weight: 600;
  padding: 0.875rem 2rem; /* 14px 32px */
  text-decoration: none;
  transition: all 200ms ease-in-out;
}

.ctaButton:hover {
  background-color: var(--cta-button-hover-bg);
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(45, 122, 79, 0.2);
}

.ctaButton:active {
  transform: translateY(0);
}

/* Reduced motion support */
@media (prefers-reduced-motion: reduce) {
  .ctaButton {
    transition: none;
    transform: none;
  }
  .ctaButton:hover {
    transform: none;
  }
}

/* Secondary variant (if needed) */
.ctaButtonSecondary {
  background-color: transparent;
  border: 2px solid var(--ifm-color-primary);
  color: var(--ifm-color-primary);
}
```

**Design Rationale**:
- Uses Docusaurus `Link` component for SPA navigation (no page reload)
- CSS transitions target <100ms response time (200ms for smooth visual)
- Accessibility: respects `prefers-reduced-motion` for users with vestibular disorders

---

## Configuration Data Structures

### Theme Configuration

**File**: `docusaurus.config.js`
**Section**: `themeConfig.colorMode` and `themeConfig.navbar`

#### Navbar Configuration

```javascript
module.exports = {
  themeConfig: {
    navbar: {
      title: 'Educational Textbooks',
      logo: {
        alt: 'Textbook Logo',
        src: 'img/logo.svg', // Optional - can use text-only
      },
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/your-org/your-repo', // Update with actual repo
          label: 'GitHub',
          position: 'right',
          target: '_blank',
          rel: 'noopener noreferrer',
        },
      ],
    },
  },
};
```

**Data Schema**:
- `type`: `'doc'` (Docusaurus doc link) or `'href'` (external link)
- `docId`: Document ID for internal links
- `position`: `'left'` or `'right'` in navbar
- `label`: Display text
- `target`, `rel`: Security attributes for external links

#### Color Mode Configuration

```javascript
module.exports = {
  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false, // Allow dark mode toggle
      respectPrefersColorScheme: true,
    },
  },
};
```

**Design Rationale**:
- Respects user's OS dark mode preference
- Dark mode support comes free with Docusaurus (out of scope for P1 but supported)

---

### CSS Custom Properties

**File**: `src/css/custom.css`
**Scope**: Global theme variables

#### Color Variables

```css
:root {
  /* Primary green theme (from research.md Decision 1) */
  --ifm-color-primary: #2D7A4F;
  --ifm-color-primary-dark: #1F5738;
  --ifm-color-primary-light: #4A9968;

  /* Hero section colors */
  --hero-bg-color: #F9FFF9;

  /* CTA button colors */
  --cta-button-bg: var(--ifm-color-primary);
  --cta-button-hover-bg: var(--ifm-color-primary-dark);

  /* Background accents */
  --ifm-background-surface-color: #E8F5E9;
}

[data-theme='dark'] {
  /* Dark mode overrides (optional - not P1 requirement) */
  --ifm-color-primary: #4A9968;
  --hero-bg-color: #1A2F1A;
}
```

#### Spacing Variables

```css
:root {
  --hero-padding-desktop: 4rem 2rem; /* 64px 32px */
  --hero-padding-mobile: 3rem 1.5rem; /* 48px 24px */
}
```

**Design Rationale**:
- All colors meet WCAG AA contrast requirements (verified in research.md)
- CSS variables enable theming without JavaScript
- Dark mode support uses Docusaurus's built-in `[data-theme]` attribute

---

## Data Flow Diagram

```
┌─────────────────────────────────────┐
│   Docusaurus Config                 │
│   (docusaurus.config.js)            │
│   - Navbar items                    │
│   - Theme colors (overridden)       │
└────────────┬────────────────────────┘
             │
             │ Provides Layout + Navbar
             ▼
┌─────────────────────────────────────┐
│   LandingPage Component             │
│   (src/pages/index.tsx)             │
│   - Hardcoded content props         │
└────────────┬────────────────────────┘
             │
             │ Passes title, subtitle, ctaText, ctaLink
             ▼
┌─────────────────────────────────────┐
│   Hero Component                    │
│   (src/components/Hero/index.tsx)   │
│   - Renders title, subtitle         │
│   - Passes CTA data to button       │
└────────────┬────────────────────────┘
             │
             │ Passes text, href
             ▼
┌─────────────────────────────────────┐
│   CTAButton Component               │
│   (src/components/CTAButton/...)    │
│   - Renders Link with styles        │
│   - Handles hover states (CSS)      │
└─────────────────────────────────────┘
```

**State Management Summary**:
- **No Redux/Context**: All components are stateless presentations
- **Props-Only Data Flow**: Parent-to-child prop passing
- **Configuration-Driven**: Content and theme controlled via config files
- **CSS-Driven Interactions**: Hover/focus states managed via CSS (no React state)

---

## Type Safety Contracts

### TypeScript Interfaces

All interfaces are defined in component files with strict typing:

```typescript
// src/components/Hero/index.tsx
export interface HeroProps {
  title: string;
  subtitle: string;
  ctaText: string;
  ctaLink: string;
}

// src/components/CTAButton/index.tsx
export interface CTAButtonProps {
  text: string;
  href: string;
  variant?: 'primary' | 'secondary';
}
```

**Validation**:
- TypeScript compiler enforces prop types at build time
- No runtime prop validation (PropTypes) needed for TypeScript projects
- Default values handled via TypeScript optional properties (`variant?: ...`)

---

## Responsive Behavior Model

### Breakpoint-Driven Layout Changes

```
┌──────────────────────────────────────────────┐
│ Mobile (320px - 767px)                       │
├──────────────────────────────────────────────┤
│ [☰] Educational Textbooks                    │  ← Hamburger menu
├──────────────────────────────────────────────┤
│                                              │
│        Learn. Grow. Excel. (32px)           │  ← Title scaled down
│   Open-source textbooks... (20px)           │  ← Subtitle scaled down
│                                              │
│      [Start Learning]                        │  ← Full-width button
│                                              │
└──────────────────────────────────────────────┘

┌──────────────────────────────────────────────┐
│ Desktop (1024px+)                            │
├──────────────────────────────────────────────┤
│ Educational Textbooks    Textbook   GitHub   │  ← Full navbar
├──────────────────────────────────────────────┤
│                                              │
│        Learn. Grow. Excel. (48px)           │  ← Full size title
│   Open-source textbooks... (28px)           │  ← Full size subtitle
│                                              │
│         [Start Learning]                     │  ← Inline button
│                                              │
└──────────────────────────────────────────────┘
```

**Responsive Strategy**:
- **Mobile-First CSS**: Base styles for mobile, media queries add desktop enhancements
- **Navbar**: Docusaurus automatically collapses to hamburger menu on mobile
- **Typography**: `rem` units scale with user font size preferences
- **Spacing**: `clamp()` CSS function for fluid spacing (optional enhancement)

---

## Accessibility Model

### Semantic HTML Structure

```html
<main>
  <section class="hero" aria-labelledby="hero-title">
    <h1 id="hero-title">Learn. Grow. Excel.</h1>
    <h2>Open-source textbooks designed for the modern learner</h2>
    <Link
      to="/docs/intro"
      className="cta-button"
      role="button"
      aria-label="Start learning with the textbook">
      Start Learning
    </Link>
  </section>
</main>
```

**Accessibility Features**:
- ✅ Semantic landmarks (`<main>`, `<section>`)
- ✅ Proper heading hierarchy (`<h1>` → `<h2>`)
- ✅ ARIA labels for screen readers (`aria-labelledby`, `aria-label`)
- ✅ Keyboard navigation (Link component supports Tab/Enter)
- ✅ Focus indicators (Docusaurus provides default focus styles)
- ✅ Color contrast (4.5:1 minimum, verified in research.md)

---

## Performance Budget Model

### Component Loading Strategy

```
Initial Page Load (Target: <1.5s FCP)
├── HTML (index.html) - 2KB
├── Critical CSS (inlined) - 5KB
├── React Runtime (Docusaurus) - 150KB (gzipped)
├── Landing Page Bundle - 3KB
│   ├── LandingPage.tsx - 1KB
│   ├── Hero.tsx - 1KB
│   └── CTAButton.tsx - 1KB
└── CSS Modules - 2KB
    ├── Hero.module.css - 1KB
    └── CTAButton.module.css - 1KB

Total Bundle Size: ~160KB (gzipped)
First Contentful Paint: <1s (target achieved via research.md Decision 7)
```

**Performance Optimizations**:
- **Zero Images**: Solid color backgrounds only (no hero image)
- **System Fonts**: No web font downloads (instant text render)
- **CSS Modules**: Automatic code splitting (Webpack)
- **No External Deps**: No axios, lodash, or animation libraries

---

## Testing Data Model

### Component Test Cases

**Hero Component Tests** (manual verification):
1. **Props Rendering**: Verify title, subtitle, and CTA text display correctly
2. **Responsive Scaling**: Check font sizes at 320px, 768px, 1024px
3. **Color Application**: Verify green theme colors applied
4. **ARIA Attributes**: Inspect `aria-labelledby` in DOM

**CTAButton Component Tests** (manual verification):
1. **Click Navigation**: Verify clicking button navigates to `/docs/intro`
2. **Hover State**: Verify background darkens and transform applies
3. **Keyboard Access**: Tab to button, press Enter, verify navigation
4. **Reduced Motion**: Enable OS setting, verify no transform/transition

**Integration Tests** (Lighthouse):
1. **Performance**: Score 90+ (FCP <1.5s, TTI <3.5s)
2. **Accessibility**: Score 95+ (contrast, ARIA, keyboard nav)
3. **SEO**: Score 90+ (meta tags, semantic HTML)

---

## File Manifest

### New Files (Created by Implementation)

```
src/
├── pages/
│   └── index.tsx                    # LandingPage component (120 lines)
├── components/
│   ├── Hero/
│   │   ├── index.tsx                # Hero component (60 lines)
│   │   └── styles.module.css        # Hero styles (80 lines)
│   └── CTAButton/
│       ├── index.tsx                # CTAButton component (40 lines)
│       └── styles.module.css        # Button styles (50 lines)
└── css/
    └── custom.css                   # MODIFIED: +30 lines (theme variables)
```

### Modified Files

```
docusaurus.config.js                 # MODIFIED: navbar items (~10 lines added)
```

**Total New Code**: ~350 lines
**Total Modified Code**: ~40 lines

---

## Dependencies

### External Dependencies (Already Installed)

```json
{
  "@docusaurus/core": "3.9.2",
  "@docusaurus/preset-classic": "3.9.2",
  "react": "18.3.1",
  "react-dom": "18.3.1"
}
```

**Zero New Dependencies**: Feature uses only existing Docusaurus packages.

### Internal Dependencies

- **Layout Component**: `@theme/Layout` (Docusaurus theme)
- **Link Component**: `@docusaurus/Link` (client-side routing)
- **Infima CSS**: Docusaurus's built-in CSS framework (responsive utilities)

---

## Constraints and Invariants

### Design Constraints

1. **No Backend**: All content is static (hardcoded in components or config)
2. **No CMS**: Title/subtitle are not editable without code changes
3. **No Analytics**: Page views/clicks not tracked (out of scope)
4. **Single Language**: English-only content (no i18n)

### Technical Invariants

1. **Stateless Components**: All components must remain pure functions (no React state)
2. **Props-Only Data**: No global state or Context API usage
3. **CSS Modules**: All component styles must use CSS modules (`.module.css`)
4. **WCAG AA Compliance**: All color combinations must maintain 4.5:1 contrast
5. **Docusaurus Compatibility**: Must work with Docusaurus 3.x architecture (no swizzling)

### Performance Invariants

1. **Bundle Size**: Total component code <5KB (uncompressed)
2. **First Contentful Paint**: Must stay <1.5s on 3G
3. **Lighthouse Score**: Performance 90+, Accessibility 95+ (regression prevention)

---

## Summary

This data model defines:
- ✅ **3 React components** with TypeScript interfaces
- ✅ **Component hierarchy** and data flow (props-only, no state)
- ✅ **CSS architecture** using modules and custom properties
- ✅ **Configuration schema** for navbar and theme
- ✅ **Responsive behavior** across 3 breakpoints
- ✅ **Accessibility model** with semantic HTML and ARIA
- ✅ **Performance budget** (<160KB total, <1s FCP)
- ✅ **Testing strategy** (manual + Lighthouse)

**Next Step**: Create JSON schema contracts in `/contracts/` directory to formalize component interfaces for tooling validation.
