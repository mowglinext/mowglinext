# Security Policy

## Reporting a Vulnerability

If you discover a security vulnerability in MowgliNext, please report it responsibly.

**Do NOT open a public issue for security vulnerabilities.**

Instead, please use [GitHub Security Advisories](https://github.com/mowglinext/mowglinext/security/advisories/new) to report the vulnerability privately.

## Scope

Security issues that are in scope:

- **Firmware**: Buffer overflows, unsafe memory access in STM32 code
- **GUI**: Authentication bypass, XSS, CSRF, injection vulnerabilities
- **Docker**: Container escape, privilege escalation, exposed secrets
- **ROS2**: Unauthorized command execution via ROS2 topics/services

## Safety Notice

MowgliNext controls a robot with spinning blades. The STM32 firmware is the safety authority for blade control — ROS2 commands are fire-and-forget. If you find any issue where software could cause unsafe physical behavior, please report it immediately.

## Supported Versions

| Version | Supported |
|---------|-----------|
| main branch | Yes |
| Other branches | No |
