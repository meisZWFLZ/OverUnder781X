# PROS
PROS is an open source operating system for the v5 brain. 

### Benefits
- Allows us to utilize modern development tools like Visual Studio Code (VSCode)
  - Builtin integration with git and github
  - VSCode permits our team to use extensions that enhance our workflow. These include:
    - [PROS](https://marketplace.visualstudio.com/items?itemName=sigbots.pros):
      automatically installs the pros toolchain and provides a gui and commands to use it 
    - [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd): 
      advanced intellisense engine for c++ using the open source clang compiler
    - [Code Spell Checker](https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker): 
      prevents spelling mistakes
    - [Git Graph](https://marketplace.visualstudio.com/items?itemName=mhutchie.git-graph):
      provides a visualization of the git commit history
    - [Doxygen Documentation Generator](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen): 
      aides in documenting code
    - [Github Copilot](https://marketplace.visualstudio.com/items?itemName=GitHub.copilot): 
      provides ai autocomplete and aides in documenting code
    - and many others
  > [!NOTE]
  > VEX does have an vexcode extension for vscode, but it is still in beta and it is not yet as sophisticated as PROS.
- Enables the use of libraries
  - [LemLib](https://github.com/LemLib/LemLib) - an open source library with a focus on odometry and motion algorithms. 
    - Odometry: enables tracking the robot's location local to the field
    - Boomerang: efficient algorithm to a move robot to a point and a heading
    - Pure Pursuit: algorithm to follow a complex path