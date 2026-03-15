process.chdir(__dirname)
process.argv.push('--host')
require('child_process').execFileSync(
  process.execPath,
  ['node_modules/vite/bin/vite.js', '--host'],
  { cwd: __dirname, stdio: 'inherit' }
)
