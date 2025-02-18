# Clean target befor take hard disk snapshot

- **CLean home directory by removing unnecessary folders manually**


```
hear-cli target copy_run_program --p clean_target_for_deployment
```

- remove wlan0 static ip 

```
rm -rf ~/.droneleaf/dynamodb
rm ~/.droneleaf/config.json
```
