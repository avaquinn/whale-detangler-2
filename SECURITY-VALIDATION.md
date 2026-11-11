## Automated Checks
|Control | Tool | Coverage / Limitations |
|---|---|---|
|Secret Scanning|GitLeaks|Scans commit history for secrets. While not direct proof of a leak, it can allow for a further investigation.|
|Dependency Scanning|OSV-Scanner|Scans dependency manifests and lockfiles. No such method exists within the project yet, but once it does, it will be integrated here.|
|SAST|Cppcheck|Static analysis of the source code. Uses warning, performance, and portability categories within its test. AVR header resolution is suppressed due to those headers being external of the project.|
|Linting|Cppcheck|Lint check of the source code. Uses style,unusedFunction for its test. It should be noted that Arduino lifecycle callbacks are known to trip false positives.|
|Arduino strucutre|Arduino Lint| Runs sketch/specification checks. Currently, missing license is the only present warning.|

## Running / Interpreting Checks
All checks run in GitHub Actions after each push.
Thus, one should review the relevant commit ID and workflow version for further information.
1. GitLeaks finding: remove the secret from the change, rotate credentials, and rerun the workflow.
2. OSV finding: identify the package, version, advisory, and patched version. Upgrade or remove depending on your situation.
If its found that no safe upgrade exists, a temporary exception may be need to be submitted.
3. CppCheck finding: investigate if its a false alarm, an Arduino lifecycle callback, or a legitimate issue.
Correct it where possible, avoid broad suppression of rules unless there is a clear reason for it.
4. Arduino Lint finding: correct sketch, filename, or metadata. Warnings do not necessarily indicate a problem, but should be addressed when possible.


## Controlled Validation Cases
Below lists examples of controlled validation cases used to test the security validation process.
|Control|Defective Case|Corrected Case|
|----|----|----|
|GitLeaks|Secret in code|Secret removed|
|OSV|Legitimate upgrade available|Upgrade performed|
|CppCheck SAST|Null pointer, uninitialized variable, out of bounds|Code removed or fixed|
|CppCheck Lint|C-style cast, unused-function|Code removed or fixed|
|Arduino Lint|Sketch folder with a different named .ino file from the folder|Corrected .ino filename|

Cases are then recorded (defective and clean) within the evidence folder and then the test cases themselves are removed.
Thus, we don't want to retain any of these test cases in the repository to ensure that the security validation process is accurate and reliable.

## Evidence sanitization
- Censor secret keys, credentials, and other sensitive information from the evidence.
- Use identifiers (IDs, package names, identifier hashs, etc) to anonymize sensitive information in the evidence.
