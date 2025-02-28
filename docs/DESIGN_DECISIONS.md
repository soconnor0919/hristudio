# HRIStudio Design Decisions Documentation

## Overview
This document captures the design decisions made during the implementation of HRIStudio. It covers various areas including UI improvements, plugin store architecture, data loading and error handling, styling conformance, and overall system architecture. This serves as a reference for developers and other stakeholders to understand the rationale behind our choices.

## 1. UI Design & User Experience
- **Conditional Rendering and User Feedback:**
  - Implemented dynamic messages in components (e.g., PluginBrowser, RobotList) to notify users when repositories or plugins are missing.
  - Clear call-to-action prompts guide users to add repositories/plugins when necessary.
- **Consistent Styling and Themes:**
  - Merged multiple globals.css files to create a cohesive, unified theme across the application.
  - Updated sidebar and top bar styling, including gradients, background colors, and hover effects to ensure visual consistency in both light and dark modes.
- **Component-Based Approach:**
  - Developed reusable, functional components (e.g., StudyCard, StudyForm, DeleteStudyButton) with TypeScript interfaces for enhanced maintainability and readability.

## 2. Plugin Store Architecture
- **Repository and Plugin Loading:**
  - Updated the plugin loading mechanism to fetch repository metadata from `repository.json` instead of `index.json`.
  - Implemented a fall-back sequence for loading individual plugin files (prioritizing `robot-plugins/plugins/{filename}` over alternative locations).
- **Error Handling:**
  - Introduced a custom `PluginLoadError` to encapsulate errors in plugin loading and provide clearer error messages.
- **Caching Strategy:**
  - Incorporated cache management with a TTL (5 minutes) to reduce frequent metadata fetching.
- **Transform Functions:**
  - Allowed registration and retrieval of transformation functions (e.g., `transformToTwist`, `transformToPoseStamped`) that convert plugin parameters into suitable payload formats.

## 3. Data Access & ORM Integration
- **Drizzle ORM:**
  - Adopted Drizzle ORM for type-safe database operations, using a custom `createTable` utility to avoid table naming conflicts.
  - Defined schemas for studies, participants, permissions, and plugin repositories with clear relationships.
- **Type-Safe Data Validation:**
  - Used Zod schemas along with TypeScript interfaces to validate data structures, ensuring integrity in plugin metadata and user data.

## 4. Authentication & Permissions
- **NextAuth & Auth.js Integration:**
  - Configured NextAuth with a credentials provider, including bcrypt-based password verification.
  - Extended session callbacks to include custom user details like name and role.
- **Role-Based Access Control:**
  - Established a clear role hierarchy (Owner, Admin, Principal Investigator, Wizard, Researcher, Observer) with distinct permissions defined in a role-permission matrix.
  - Enforced visibility and access controls for sensitive information based on user roles.

## 5. Next.js Architecture and Component Providers
- **Next.js 15 App Router:**
  - Leveraged the new App Router to prioritize server components over client components, reducing reliance on `useEffect` and client-side state for data fetching.
- **Global Providers:**
  - Wrapped the application with multiple providers (ThemeProvider, PluginStoreProvider, StudyProvider) to supply context and maintain consistent application state.

## 6. Additional Design Considerations
- **Error Logging and Debugging:**
  - Enhanced error logging throughout the plugin store and repository loading processes to aid in troubleshooting.
- **Naming Conventions and File Structure:**
  - Maintained a clear naming convention for directories (lowercase with dashes) and reused descriptive interface names to ensure clarity and consistency.

## Conclusion
These design decisions were made to build a robust, scalable, and user-friendly platform for managing human-robot interaction studies. By emphasizing type safety, modularization, and thorough error handling, HRIStudio is well-equipped for both current needs and future enhancements. Future work may focus on extending experimental features and integrating advanced analytics tools. 