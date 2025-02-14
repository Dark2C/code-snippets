@echo off
setlocal enabledelayedexpansion
(
echo FROM alpine/openssl AS builder
echo RUN openssl ecparam -genkey -name prime256v1 -noout -out private.pem
echo RUN openssl ec -in private.pem -pubout -out public.pem

echo FROM scratch
echo COPY --from=builder private.pem /private.pem
echo COPY --from=builder public.pem /public..pem
) > Dockerfile
docker build --output type=tar,dest=keys.tar -f Dockerfile .
del Dockerfile
