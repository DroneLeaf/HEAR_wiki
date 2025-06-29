# Getting Started with Petal App Manager

This guide walks you through setting up the necessary repositories and development environment for working with the Petal App Manager.

## Initial Setup

To set up the required repositories, run:

```bash
hear-cli local_machine run_program --p petal_app_manager_prepare_sitl
```

This command will set up the following repository structure:

```
petal-app-dev/
├── mavlink/
│   └── pymavlink/
├── petal-app-manager/
├── petal-flight-log/
└── petal-hello-world/
```

## Dependencies Management

Each repository manages its dependencies through its own `pyproject.toml` file. When developing or extending functionality:

- Make sure to check the respective repository's dependencies
- Add any new dependencies to the appropriate `pyproject.toml` file

> [!NOTE]
> For details read [this readme file](https://github.com/DroneLeaf/petal-app-manager/blob/main/petals.md)

## Plugin Development

When developing plugins, you'll need to modify the corresponding plugin file:

```
src/petal-name/plugin.py
```

Adjust the implementation according to your application requirements.

## Working with MAVLink Messages

### Adding New MAVLink Messages

1. Modify the message definitions in:
    ```
    mavlink/message_definitions/v1.0
    ```

2. Ensure correct XML formatting:
    ```bash
    ./scripts/format_xml.sh
    ```

3. Generate Python bindings:
    ```bash
    cd pymavlink
    python setup.py build
    ```

This process will generate leaf message definitions for Python under `dialects/v10/*.xml`.

## Releasing pymavlink to PyPI

When you need to publish a new version of pymavlink:

1. Modify the version number in `mavlink/pymavlink/__init__.py`
    - Always increment the minor version

2. Create and push a git tag:
    ```bash
    git tag vx.x.x
    git push origin vx.x.x
    ```

This will trigger the release process for publishing to PyPI.