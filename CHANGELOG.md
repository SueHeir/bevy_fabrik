# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2025-01-28

### Changed

- Updated bevy dependency from 0.14 to 0.15

## [0.1.1] - 2024-11-01

### Added

- `IkSystem::Solve` system set for scheduling

### Changed

- Moved solving IK from `Update` to `PostUpdate` schedule, after `GlobalTransform` propagation
- Removed `GlobalTransform` manual writes in the basic example

## [0.1.0] - 2024-10-22

Initial release
