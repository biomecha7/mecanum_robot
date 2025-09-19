# Contributing to Mecanum Robot Controller

Thank you for your interest in contributing to the Mecanum Robot Controller project! This document provides guidelines and information for contributors.

## 🚀 Getting Started

### Prerequisites
- PlatformIO or Arduino IDE
- ESP32 development environment
- Basic understanding of robotics and embedded systems
- Git and GitHub account

### Development Setup

1. **Fork the repository**
   ```bash
   git clone https://github.com/yourusername/mecanum_robot.git
   cd mecanum_robot
   ```

2. **Install dependencies**
   ```bash
   pio lib install
   ```

3. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

## 📋 Contribution Guidelines

### Code Style

- **Naming Conventions**:
  - Classes: `PascalCase` (e.g., `MotionController`)
  - Functions and variables: `camelCase` (e.g., `updateSensors`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `WHEELBASE_INCHES`)
  - Files: Match class names (e.g., `MotionController.h`)

- **Formatting**:
  - Use 2 spaces for indentation
  - Maximum line length: 100 characters
  - Consistent brace placement
  - Clear, descriptive variable names

- **Documentation**:
  - Document all public interfaces
  - Include parameter descriptions
  - Add usage examples for complex functions
  - Update README.md for API changes

### Commit Messages

Use clear, descriptive commit messages:

```
feat: Add emergency stop clear mechanism

- Implement SELECT+START button combination
- Add timeout-based ESTOP clearing
- Update Supervisor state machine logic

Fixes #123
```

**Conventional Commits format**:
- `feat:` New features
- `fix:` Bug fixes
- `docs:` Documentation changes
- `style:` Code style changes
- `refactor:` Code refactoring
- `test:` Adding or updating tests
- `chore:` Maintenance tasks

### Pull Request Process

1. **Create a feature branch** from `main`
2. **Make your changes** following the code style guidelines
3. **Add tests** for new functionality
4. **Update documentation** as needed
5. **Test your changes** thoroughly
6. **Submit a pull request** with a clear description

### Pull Request Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
- [ ] Tested on hardware
- [ ] Unit tests pass
- [ ] Integration tests pass

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Documentation updated
- [ ] Tests added/updated
```

## 🧪 Testing

### Hardware Testing
- Test all changes on actual hardware
- Verify sensor readings and motor responses
- Check emergency stop functionality
- Validate communication protocols

### Code Quality
- No compiler warnings
- Proper error handling
- Memory management (no leaks)
- Real-time performance maintained

### Performance Requirements
- Control loop: 100Hz (10ms)
- Sensor updates: Encoders 100Hz, IMU 200Hz
- Latency: <5ms command to motor response

## 🐛 Bug Reports

When reporting bugs, please include:

1. **Environment**:
   - Hardware version
   - Software version
   - PlatformIO/Arduino IDE version

2. **Steps to reproduce**:
   - Clear, numbered steps
   - Expected behavior
   - Actual behavior

3. **Additional context**:
   - Serial output logs
   - Screenshots if applicable
   - Related issues

## 💡 Feature Requests

For new features:

1. **Check existing issues** first
2. **Provide use case** and motivation
3. **Describe implementation** approach
4. **Consider backwards compatibility**

## 🏗️ Architecture Guidelines

### Design Principles
- **Single Responsibility**: Each class has one clear purpose
- **Dependency Injection**: Use constructor injection for dependencies
- **Interface Segregation**: Small, focused interfaces
- **Real-time Safety**: No blocking operations in control loops

### Task Design
- **Separate concerns**: Sensors, control, communication
- **Priority-based scheduling**: Critical tasks get higher priority
- **Queue-based communication**: Avoid shared state
- **Graceful degradation**: Handle component failures

### Safety Requirements
- **Emergency stop**: Always available and functional
- **Watchdog timers**: Prevent system lockups
- **Fault detection**: Monitor sensor and motor health
- **Graceful shutdown**: Safe power-down procedures

## 📚 Documentation

### Code Documentation
- Use Doxygen-style comments for public APIs
- Include parameter types and return values
- Provide usage examples for complex functions
- Document timing requirements and constraints

### User Documentation
- Update README.md for new features
- Add troubleshooting guides for common issues
- Include hardware setup instructions
- Provide configuration examples

## 🤝 Community Guidelines

### Be Respectful
- Use welcoming and inclusive language
- Respect different viewpoints and experiences
- Accept constructive criticism gracefully
- Focus on what's best for the community

### Be Collaborative
- Help others learn and grow
- Share knowledge and best practices
- Contribute to discussions constructively
- Recognize contributions from others

## 📞 Getting Help

- **GitHub Issues**: For bugs and feature requests
- **GitHub Discussions**: For questions and general discussion
- **Email**: For sensitive or private matters

## 🏆 Recognition

Contributors will be recognized in:
- README.md acknowledgments
- Release notes
- Project documentation

Thank you for contributing to the Mecanum Robot Controller project!
