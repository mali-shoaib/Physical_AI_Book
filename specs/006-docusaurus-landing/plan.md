# Implementation Plan: Docusaurus Landing Page UI

**Branch**: `006-docusaurus-landing` | **Date**: 2024-12-23 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/006-docusaurus-landing/spec.md`

## Summary

Build a custom React-based landing page for the existing Docusaurus textbook website. The landing page will feature a hero section with title, subtitle, and CTA button, implemented with a green education-themed design. The page will integrate with the existing Docusaurus navigation structure, adding navbar links for "Textbook" and "GitHub", and will be fully responsive across mobile, tablet, and desktop viewports.

**Technical Approach**: Customize Docusaurus by creating a custom homepage React component (`src/pages/index.tsx`), override theme colors via `docusaurus.config.js` and custom CSS for the green theme, and leverage Docusaurus's built-in responsive design system for mobile adaptation.

## Technical Context

**Language/Version**: TypeScript 5.x / React 18.x (Docusaurus 3.x requirement)
**Primary Dependencies**: Docusaurus 3.9.2 (already installed), @docusaurus/preset-classic, React 18.3.1, react-dom 18.3.1
**Storage**: N/A (static site, no backend storage)
**Testing**: Manual testing via local dev server, Lighthouse CI for performance/accessibility validation
**Target Platform**: Modern web browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+), mobile responsive
**Project Type**: Web (frontend-only static site)
**Performance Goals**: Lighthouse performance score 90+, accessibility score 95+, First Contentful Paint <1.5s
**Constraints**: Must work within existing Docusaurus 3.x architecture, maintain compatibility with existing docs structure, green theme with WCAG 2.1 AA contrast ratios (4.5:1 minimum)
**Scale/Scope**: Single landing page component, 4 React components (Hero, CTA Button, custom navbar elements), ~200-300 lines of TypeScript/JSX code

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Status**: ✅ **NOT APPLICABLE** - This is a frontend UI feature with no backend architecture

This feature is purely presentational UI work on an existing Docusaurus site. There are no:
- Libraries to create (using existing Docusaurus framework)
- CLI interfaces (web UI only)
- Test-first requirements (UI design work, validated via Lighthouse)
- Backend services or APIs
- Data persistence or state management beyond React component state

The existing project follows standard Docusaurus patterns. This feature adds custom React components and theme configuration within that framework.

## Project Structure

### Documentation (this feature)

```text
specs/006-docusaurus-landing/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (design decisions)
├── data-model.md        # Phase 1 output (component structure)
├── quickstart.md        # Phase 1 output (implementation guide)
├── contracts/           # Phase 1 output (prop types, theme config)
│   ├── hero-component.schema.json
│   ├── theme-config.schema.json
│   └── navbar-config.schema.json
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Existing Docusaurus structure (relevant parts)
src/
├── pages/
│   └── index.tsx        # NEW: Custom landing page component (replaces default)
├── components/
│   ├── Hero/
│   │   ├── index.tsx    # NEW: Hero section component
│   │   └── styles.module.css  # NEW: Hero section styles
│   └── CTAButton/
│       ├── index.tsx    # NEW: CTA button component
│       └── styles.module.css  # NEW: CTA button styles
└── css/
    └── custom.css       # MODIFIED: Add green theme variables

docusaurus.config.js     # MODIFIED: Update navbar, theme colors
static/                  # MODIFIED: Add any hero images/assets
docs/                    # EXISTING: Textbook documentation (unchanged)
```

**Structure Decision**: Using standard Docusaurus frontend structure. The landing page is implemented as `src/pages/index.tsx` which Docusaurus automatically serves at the root URL. Reusable components (Hero, CTAButton) follow React component conventions with co-located CSS modules. Theme customization uses Docusaurus's built-in theming system via `docusaurus.config.js` and custom CSS variables.

## Complexity Tracking

> This section is empty because there are no Constitution violations to justify.

## Phase 0: Research & Design Decisions

**Objective**: Resolve all technical unknowns and establish design patterns for the green education theme, responsive breakpoints, and Docusaurus customization approach.

**Research Tasks**:

1. **Green Education Theme Color Research**
   - Research modern education website color palettes (EdX, Coursera, Khan Academy)
   - Define primary/secondary/accent green shades with WCAG AA compliance
   - Document hex codes and usage patterns

2. **Docusaurus Theme Customization Best Practices**
   - Research `docusaurus.config.js` theme color overrides
   - Explore CSS custom properties approach for theme variables
   - Determine if swizzling theme components is needed

3. **Responsive Breakpoint Standards**
   - Confirm mobile (320px-767px), tablet (768px-1023px), desktop (1024px+) breakpoints
   - Research CSS Grid vs Flexbox best practices for hero layout
   - Explore Docusaurus responsive utilities

4. **Hero Section Layout Patterns**
   - Research hero section composition (centered vs left-aligned)
   - Determine optimal title/subtitle typography scale
   - Explore CTA button placement and sizing best practices

5. **Docusaurus Navbar Customization**
   - Research how to add custom navbar items
   - Determine GitHub icon/link implementation approach
   - Explore mobile hamburger menu customization options

**Output**: `research.md` documenting all design decisions, color specifications, responsive patterns, and implementation approach

## Phase 1: Data Model & Contracts

**Objective**: Define component interfaces, theme configuration schema, and implementation structure.

### Data Model (`data-model.md`)

**Component Structure**:

1. **LandingPage Component** (`src/pages/index.tsx`)
   - Root component rendering hero and layout
   - State: None (stateless presentation)
   - Props: None (uses config from docusaurus.config.js)

2. **Hero Component** (`src/components/Hero/index.tsx`)
   - Props: `{ title: string, subtitle: string, ctaText: string, ctaLink: string }`
   - State: None (stateless presentation)
   - Children: CTAButton component

3. **CTAButton Component** (`src/components/CTAButton/index.tsx`)
   - Props: `{ text: string, href: string, variant?: 'primary' | 'secondary' }`
   - State: hover (managed via CSS pseudo-classes)

4. **Theme Configuration** (`docusaurus.config.js`)
   - Color tokens: primary, primaryDark, primaryLight (green shades)
   - Navbar items: { label, href, position, external }
   - Hero content: title, subtitle, button text

### Contracts (`/contracts/`)

1. **hero-component.schema.json**: TypeScript prop interface for Hero component
2. **theme-config.schema.json**: Theme color and typography configuration
3. **navbar-config.schema.json**: Navbar item structure for Textbook and GitHub links

### Quickstart (`quickstart.md`)

Step-by-step guide for:
1. Creating custom landing page component
2. Implementing Hero and CTAButton components
3. Configuring green theme colors
4. Adding navbar links
5. Testing responsive behavior
6. Running Lighthouse audits

**Output**: `data-model.md`, `/contracts/*.json`, `quickstart.md`

## Phase 2: Implementation Tasks

**Note**: Detailed task breakdown will be created by `/sp.tasks` command after this planning phase.

**High-Level Implementation Phases**:

1. **Phase A: Core Landing Page Structure** (P1 - MVP)
   - Create `src/pages/index.tsx` with basic layout
   - Implement Hero component with title, subtitle
   - Implement CTAButton component
   - Add navbar "Textbook" and "GitHub" links to config
   - Verify basic functionality (user can navigate)

2. **Phase B: Green Theme Styling** (P2)
   - Define green color palette in custom.css
   - Override Docusaurus theme colors in config
   - Apply green theme to CTA button
   - Implement hover states with green accents
   - Verify WCAG AA contrast ratios

3. **Phase C: Responsive Mobile Adaptation** (P3)
   - Add CSS media queries for mobile/tablet breakpoints
   - Implement responsive typography scaling
   - Test mobile navbar hamburger menu
   - Verify layout on 320px-2560px viewports
   - Fix any layout shifts or overflow issues

4. **Phase D: Polish & Performance** (Final)
   - Run Lighthouse audits
   - Optimize asset loading (images, fonts)
   - Add loading states if needed
   - Verify accessibility (ARIA labels, semantic HTML)
   - Cross-browser testing

**Testing Strategy**:
- Manual testing: Visual review on Chrome DevTools device emulator
- Performance: Lighthouse CI (target: 90+ performance, 95+ accessibility)
- Cross-browser: Manual testing on Chrome, Firefox, Safari
- Responsive: Test on actual mobile device or BrowserStack

## Dependencies & Risks

### External Dependencies
- **Docusaurus 3.9.2**: Already installed, no upgrade needed
- **React 18.3.1**: Already installed, compatible with TypeScript
- **No external libraries**: Using pure CSS for styling (avoiding dependency bloat)

### Technical Risks

1. **Risk**: Green theme colors may not meet WCAG AA contrast requirements
   - **Mitigation**: Use WebAIM contrast checker during design phase
   - **Impact**: Low (can adjust color shades)

2. **Risk**: Custom landing page may conflict with Docusaurus routing
   - **Mitigation**: Use standard `src/pages/index.tsx` convention
   - **Impact**: Low (well-documented Docusaurus pattern)

3. **Risk**: Mobile navbar may not adapt properly on small screens
   - **Mitigation**: Test early on 320px viewport, use Docusaurus responsive utilities
   - **Impact**: Medium (requires careful CSS)

4. **Risk**: Lighthouse performance score may fall below 90 due to asset loading
   - **Mitigation**: Lazy load images, use modern image formats (WebP), inline critical CSS
   - **Impact**: Medium (may require optimization iteration)

### Implementation Assumptions

1. Existing Docusaurus site is functional (confirmed - currently running on port 8080)
2. GitHub repository URL is available in existing config or env variable
3. "Textbook" link points to `/docs/` (main documentation entry)
4. Hero content (exact title/subtitle text) will be determined during implementation
5. No backend API needed - all content is static
6. Browser support: Last 2 versions of major browsers (per Docusaurus defaults)

## Next Steps

1. ✅ **Completed**: Specification (`/sp.specify`)
2. ✅ **Current**: Planning (`/sp.plan`) - this document
3. ⏭️ **Next**: Execute Phase 0 research to generate `research.md`
4. ⏭️ **Then**: Execute Phase 1 design to generate `data-model.md`, `contracts/`, `quickstart.md`
5. ⏭️ **After Planning**: Run `/sp.tasks` to generate task breakdown

**Command to proceed**: Ready to execute Phase 0 and Phase 1 within this planning session.
