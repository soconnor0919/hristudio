module.exports = {
  "extends": [".eslintrc.cjs"],
  "rules": {
    // Only enable the rule we want to autofix
    "@typescript-eslint/prefer-nullish-coalescing": "error"
  }
};