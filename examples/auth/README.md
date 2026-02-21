# Authentication Demo

Demonstrates JWT authentication with Agent ROS Bridge.

## Running

```bash
export JWT_SECRET=$(openssl rand -base64 32)
docker-compose up
```

## Authentication

Generate a token:
```bash
docker-compose exec auth-demo python3 scripts/generate_token.py --user admin --roles admin
```

Connect with token:
```bash
wscat -c "ws://localhost:8768?token=YOUR_TOKEN"
```

## Features

- JWT token validation
- Role-based access control
- API key authentication
- Token expiration
