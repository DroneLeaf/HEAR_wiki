# `Data Life Cycle Management` Parocess


DLM process is the a structured approach to handling data as it moves through various stages, from creation and initial storage to eventual archival or deletion


## Stages of DLM

- Creation and Acquisition: Data is generated or collected.
When a new organization or client start integrate with our software, the system start to generate all default and neccessary data and then collect the other data from the user through all client apps he has access to.

- Storage: Data is secuerly stored in dynamodb (cloud,local)
- Usage and Access: Data is accessed through `api` service 
(express api), this service is the middleware between apps and data even stored on either cloud or local branches.

  Data can be accessed directly through `dynamodb manager` web app for cloud and local dynamodb and take full access .
  so you can edit directly without `api`.


## Prepare target machine for local data store and access

All steps will done using `hear-cli`
- Docker Install (`Run Once on target`)

```bash
hear-cli target copy_run_program --p docker_install
```

- Node.js Install (`Run Once on target`)

```bash
hear-cli target copy_run_program --p node_install
```

- Init Ecr Pull Profile

```bash
hear-cli target copy_run_program --p init_ecr_pull_profile
```

- Data lifecycle prepare

```bash
hear-cli target copy_run_program --p data_lifecycle_prepare
```

- Controller Dashboard prepare

```bash
hear-cli target copy_run_program --p controller_dashboard_prepare
```

- Delete aws profiles (After finishing sync process)

```bash
hear-cli target copy_run_program --p delete_aws_profiles
```


- Init Sync Profile

```bash
hear-cli target copy_run_program --p init_sync_profile
```




Controller Dashboard access at :
http://localhost

Dynamodb Manager access at :
http://localhost:8080

Api access at :
http://localhost:3000



## For Service Team
- Sync cloud to local dynamodb

```bash
hear-cli target copy_run_program --p sync_cloud_to_local_dynamoDB
```


- Sync local to cloud dynamodb (if needed)

```bash
hear-cli target copy_run_program --p sync_local_to_cloud_dynamodb
```

