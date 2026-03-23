# Homepage Screenshots

Add your app screenshots here. The homepage will display them automatically.

## Required Screenshots

1. **experiment-designer.png** - Visual experiment designer showing block-based workflow
2. **wizard-interface.png** - Wizard execution interface with trial controls
3. **dashboard.png** - Study dashboard showing experiments and trials

## Recommended Size

- Width: 1200px
- Format: PNG or WebP
- Quality: High (screenshot at 2x for retina displays)

## Preview in Browser

After adding screenshots, uncomment the `<Image>` component in `src/app/page.tsx`:

```tsx
<Image
  src={screenshot.src}
  alt={screenshot.alt}
  fill
  className="object-cover transition-transform group-hover:scale-105"
/>
```
