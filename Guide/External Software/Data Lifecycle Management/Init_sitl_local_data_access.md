DynamoDB integration steps 



- ## Prepare target machine for local data store and access

- Pull the latest `HEAR_CLI` 

All steps will be done using `hear-cli`
- Docker Install (`Run Once on target`)

```bash
hear-cli local_machine run_program --p docker_install
```

- Node.js Install (`Run Once on target`)

```bash
hear-cli local_machine run_program --p node_install
```

- Init Ecr Pull Profile

```bash
hear-cli local_machine run_program --p init_ecr_pull_profile
```

- Init Sync Profile

```bash
hear-cli local_machine run_program --p init_sync_profile
```

- Data lifecycle prepare

```bash
hear-cli local_machine run_program --p data_lifecycle_prepare
```

- Controller Dashboard prepare

```bash
hear-cli local_machine run_program --p controller_dashboard_prepare
```



- open the `controller dashboard`: http://localhost in browser
- copy the `Machine ID` from `Device Information` section


- open `client app` types page : https://fly.droneleaf.io/client/drone-types
- Press `Add Drone Type` Button and complete the process

- open `client app` drones page : https://fly.droneleaf.io/client/drones
- Press `Add Drone` Button, paste the machineId you copied from the controller dashboard , select the type you created before then complete the process.


- 