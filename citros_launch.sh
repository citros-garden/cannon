docker run -it -e CITROS_ENVIRONMENT=CLUSTER -e JOB_COMPLETION_INDEX=0 -e CITROS_DOMAIN=dev1.citros.io --mount type=bind,source="$(pwd)"/.citros/auth,target=/root/.citros/auth,readonly cannon citros run -b a519b502-e275-4ccc-90d7-9a2f6163c8d8 -i JOB_COMPLETION_INDEX -n "name" -m "message" -v -d

docker run -it -e CITROS_ENVIRONMENT=CLUSTER -e JOB_COMPLETION_INDEX=0 --mount type=bind,source="$(pwd)"/.citros/auth,target=/root/.citros/auth,readonly cannon /bin/bash



