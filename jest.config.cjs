module.exports = {
  testEnvironment: 'jsdom',
  testMatch: ['**/tests/unit/**/*.test.js', '**/tests/unit/**/*.test.mjs'],
  setupFilesAfterEnv: [],
  collectCoverage: false,
  transform: {
    '^.+\\.(js|jsx|mjs)$': 'babel-jest',
  },
};


