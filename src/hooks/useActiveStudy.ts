/**
 * @file useActiveStudy.ts
 *
 * Legacy placeholder for the deprecated `useActiveStudy` hook.
 *
 * This file exists solely to satisfy lingering TypeScript project
 * service references (e.g. editor cached import paths) after the
 * migration to the unified `useSelectedStudyDetails` hook.
 *
 * Previous responsibilities:
 * - Exposed the currently "active" study id via localStorage.
 * - Partially overlapped with a separate study context implementation.
 *
 * Migration:
 * - All consumers should now import `useSelectedStudyDetails` from:
 *     `~/hooks/useSelectedStudyDetails`
 * - That hook centralizes selection, metadata, counts, and role info.
 *
 * Safe Removal:
 * - Once you are certain no editors / build artifacts reference this
 *   path, you may delete this file. It is intentionally tiny and has
 *   zero runtime footprint unless mistakenly invoked.
 */

/**
 * @deprecated Use `useSelectedStudyDetails()` instead.
 * Legacy no-op placeholder retained only to satisfy stale references.
 * Returns a neutral object so accidental invocations are harmless.
 */
export function useActiveStudy(): DeprecatedActiveStudyHookReturn {
  return { studyId: null };
}

/**
 * Type alias maintained for backward compatibility with (now removed)
 * code that might have referenced the old hook's return type.
 * Kept minimal on purpose.
 */
export interface DeprecatedActiveStudyHookReturn {
  /** Previously the active study id (now: studyId in useSelectedStudyDetails) */
  studyId: string | null;
}

export default useActiveStudy;
