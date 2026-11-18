cat << 'EOF' > README.md
# Unitree Go1 Hackathon Solution

## Сборка Docker-образа
```bash
docker buildx build --platform linux/amd64 -t go1-solution:latest .
