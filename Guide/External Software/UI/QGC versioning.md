# Overview
This document describes how to the QGC version in the QGC app is controlled.

## How versioning is fetched in QGC
(*Note:* Analysis based on version https://github.com/mavlink/qgroundcontrol/releases/tag/v4.4.3)

In the file `cmake/Git.cmake` the QGC_APP_VERSION_STR and QGC_APP_VERSION variables are fetched based on the commands:

1- `git describe --always --tags`
2- `git describe --always --abbrev=0`

for which GitHub explanations are provided below.

-----------

### Breakdown of `git describe --always --tags`

The `git describe` command provides a human-readable identifier for a commit, usually based on the most recent tag and the number of commits since that tag. The command can be modified with various flags to change its behavior. 

The specific command `git describe --always --tags` consists of a few flags that affect its output. Here’s a breakdown of what each part of the command does:

#### 1. **`git describe`**:
This command outputs a description of the current commit based on the most recent tag and the number of commits since that tag.

Example of normal output:
```
v1.0.0-2-g2414721
```
- `v1.0.0` is the most recent tag.
- `2` is the number of commits since that tag.
- `g2414721` is the abbreviated commit hash.

#### 2. **`--always`**:
This flag tells Git to **always return a description**, even if there is no tag in the commit history.
- Without this flag, `git describe` would fail if no tags are present. The `--always` flag ensures that the command outputs a description even in the absence of tags, usually falling back to the commit hash.
- If no tags exist, the output will be the full commit hash (or an abbreviated version depending on other flags).

Example output when no tags exist:
```
2414721
```

#### 3. **`--tags`**:
This flag tells Git to use **any type of tag**, not just annotated tags. Normally, `git describe` uses only annotated tags by default. The `--tags` flag expands the search to include both annotated and lightweight tags.
- **Annotated tags** are full Git objects with additional metadata, such as the tagger's name and email, and the date.
- **Lightweight tags** are simpler references to commits without additional metadata.

By including `--tags`, the command will try to find the most recent tag (whether annotated or lightweight) to base its description on.

#### Summary of What `git describe --always --tags` Does:

- **With tags**: It outputs the most recent tag, followed by the number of commits since that tag and the abbreviated commit hash.
  - Example: `v1.0.0-2-g2414721`
  
- **Without tags**: It will fall back to the commit hash if no tags are present, providing a fallback description with the commit ID.
  - Example: `2414721`

### Example Scenarios:

1. **With Annotated or Lightweight Tags:**

   If the latest tag is `v1.0.0`, and there are 3 commits since that tag:
   ```
   $ git describe --always --tags
   v1.0.0-3-g2414721
   ```

2. **Without Any Tags:**

   If there are no tags in the history, the command will output the commit hash:
   ```
   $ git describe --always --tags
   2414721
   ```

### Summary:

- **`--always`** ensures a description is always returned, even if no tags are present, by falling back to the commit hash.
- **`--tags`** allows Git to use both annotated and lightweight tags, expanding the scope for finding the most recent tag.
- **When there are tags**, it will return the most recent tag with the number of commits since that tag, followed by the abbreviated commit hash.
- **When there are no tags**, it will output the commit hash.

In essence, `git describe --always --tags` helps you easily identify the current state of the code relative to the most recent tag, while ensuring that a description is returned even if no tags are present.

-----------

### Breakdown of `git describe --always --abbrev=0`

The `git describe` command is used in Git to retrieve a human-readable identifier for a specific commit, usually based on the most recent tag in the commit history. The command can help you quickly identify a commit by referencing a tag and the number of commits since that tag, making it easier to track the state of your code.

The specific command you've mentioned, `git describe --always --abbrev=0`, consists of a few flags that modify its behavior. Here's what each part of the command does:

#### 1. **`git describe`**:
This command outputs a description of the current commit, typically based on the most recent tag and the number of commits since that tag.
   
Example of normal output:
```
v1.0.0-2-g2414721
```
This output means that the most recent tag is `v1.0.0`, and the current commit is 2 commits ahead of that tag, with `g2414721` being the commit hash.

#### 2. **`--always`**:
This flag tells Git to **always return a description**, even if there is no tag in the commit history.
- If there are no tags in the history, `git describe` normally fails. The `--always` flag prevents this by falling back to using the commit hash itself.
- This means if no tags exist, the output will be the full commit hash (or an abbreviated version, depending on the other flags).
   
Example output when no tags exist:
```
2414721
```

#### 3. **`--abbrev=0`**:
This flag controls the abbreviation length of the commit hash.
- By default, `git describe` will shorten the commit hash to a few characters. The `--abbrev=0` flag means that **no abbreviation** is applied, and it will show only the closest tag without including the hash part or the number of commits since the tag.

Example output without `--abbrev=0`:
```
v1.0.0-2-g2414721
```

Example output **with** `--abbrev=0`:
```
v1.0.0
```
The result is just the most recent tag, without the additional commit count or hash part.

### Summary of What `git describe --always --abbrev=0` Does:

- **If a tag exists**: It outputs the most recent tag.
  - Example: `v1.0.0`
  
- **If no tag exists**: It will output the commit hash (without abbreviation) instead.
  - Example: `2414721`

In essence, `git describe --always --abbrev=0` gives you a **clean tag name** if there is a tag in the history, or the **commit hash** if there isn't.