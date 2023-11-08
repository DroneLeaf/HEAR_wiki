# Coding Guideline
**Document Scope**: Writing source code.
## Design Philosophy​

- User friendliness, mainly developers.​

- Explicit over implicit.​

- Ease of tracing and debugging.​

- The law of least surprise.

- Single responsibility of element, sub-systems, and systems.​

- Reusability.​

- Open for extension, closed for modifications.​

- Service oriented architecture with standardized intra-communication.​

## Acronyms

- ME: Mission Element

## Guidelines
**Always:**
- use override keyword whenever applicable
- one class per hpp/cpp file
- Declaration in hpp, implementation in cpp. Even empty functions.
- Use #ifdef for hardware dependent code in .hpp, exclude .cpp files from Cmake. Never use #define but use ‘Configurations.cmake’ adjacent to the root CmakeLists.txt. _(Temporary provision)_
- Never use #define outside ‘Configurations.cmake’.
- Use setters and getters to access class member variables. For example, for a private `bool en_logging` use `void setLogging(bool en_logging)` and `bool getLoggging()`. Never make `bool en_logging` public.

   

**Never:**
- use for/while/if statements without enclosing curly brackets
- use ‘using std’  
  
**Should:**
- use ‘#pragma once’ as include guard  
    
## Naming Conventions

**Variables**
- Use suffix _ptr for pointer variables

## HEAR specific

**Always:**
- For MEs set all async input ports as thread-safe.
- Never leave CLI output code temporarily written at the development phase.
- Use Logger class for logging.