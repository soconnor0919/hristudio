"use client";

export function ThemeScript() {
  return (
    <script
      dangerouslySetInnerHTML={{
        __html: `
          (function() {
            function getThemePreference() {
              if (typeof localStorage !== 'undefined' && localStorage.getItem('hristudio-theme')) {
                return localStorage.getItem('hristudio-theme');
              }
              return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
            }

            function setTheme(theme) {
              if (theme === 'system' || theme === null) {
                theme = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
              }

              // Add theme-changing class to disable transitions
              document.documentElement.classList.add('theme-changing');

              document.documentElement.classList.remove('light', 'dark');
              document.documentElement.classList.add(theme);
              document.documentElement.style.colorScheme = theme;

              // Remove theme-changing class after a brief delay
              setTimeout(() => {
                document.documentElement.classList.remove('theme-changing');
              }, 10);
            }

            setTheme(getThemePreference());

            // Listen for system theme changes
            const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
            mediaQuery.addEventListener('change', (e) => {
              const storedTheme = localStorage.getItem('hristudio-theme');
              if (storedTheme === 'system' || !storedTheme) {
                setTheme('system');
              }
            });
          })();
        `,
      }}
    />
  );
}
