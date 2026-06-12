# Feature Specification: Streets, Intersections, and Building Color Enhancement

**Feature Branch**: `002-streets-intersections`

**Created**: 2026-06-12

**Status**: Draft

**Input**: User description: "adjust the color of the buildings vs other items you will implement now. if you do streets next you will need to be intelligent to place good street contours along with intelligently adding traffic lights at intersections with crosswalks"

## Clarifications

### Session 2026-06-12

- Q: What color scheme should buildings use to visually distinguish them from upcoming street and intersection elements? → A: Buildings use a warm earth-tone palette (terracotta/brick/sandstone hues) contrasting with cool grey/charcoal streets. Color varies subtly by building height within a controlled warm range distinct from ground elements.
- Q: What LiDAR source data should be used for street extraction? → A: Ground-class LiDAR points (classification code 2) and the existing DTM terrain mesh provide the base surface for extracting road contours, centerlines, and intersection zones.
- Q: How should traffic lights be placed at intersections? → A: Traffic lights appear at all intersections where two or more road segments meet. Each approach direction gets a signal pole with red/yellow/green indicators. Crosswalks connect adjacent corners of intersections with striped path markings.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Viewer Sees Streets with Proper Contours (Priority: P1)

A user opens the web viewer and sees road surfaces overlaid on the terrain, following natural contours of the campus road network. Streets appear as flat or gently sloping surfaces with realistic widths and edge markings. Street coloring (cool greys) contrasts visibly with terrain and building colors.

**Why this priority**: Streets form the connective tissue of the digital twin that bridges buildings together. Without roads, the scene lacks navigational context and realism.

**Independent Test**: Open the viewer with streets layer enabled. Verify road surfaces follow the terrain DTM and visible paths on the aerial imagery. Toggle streets layer independently to confirm it renders as a separate layer group.

**Acceptance Scenarios**:

1. **Given** the viewer has loaded terrain and buildings, **When** the street extraction has been run, **Then** road centerlines and surfaces appear in the scene following the campus road network visible in the aerial imagery.
2. **Given** the viewer with streets visible, **When** the user toggles a "Streets" layer checkbox off, **Then** all street geometry disappears independently of buildings and terrain.
3. **Given** the viewer, **When** the user navigates to areas between known buildings, **Then** road surfaces bridge those gaps with continuous, connected geometry.

---

### User Story 2 - Intersections Display Traffic Controls and Crosswalks (Priority: P2)

A user viewing a campus intersection sees traffic signal poles with colored indicators and striped crosswalk markings connecting opposite corners. Intersections are determined by road network topology (where two or more road segments meet) rather than manually placed.

**Why this priority**: Intersections are critical navigational landmarks. Traffic signals and crosswalks provide realistic context that makes the digital twin usable for pedestrian and vehicle simulation.

**Independent Test**: Navigate the viewer to a known campus intersection (e.g., near a major building cluster). Verify at least one traffic signal pole per approach direction, correctly oriented toward oncoming traffic. Verify crosswalk stripes connect adjacent intersection corners.

**Acceptance Scenarios**:

1. **Given** the street network has been extracted, **When** the viewer renders an intersection where at least two road segments meet, **Then** traffic signal poles with red/yellow/green indicator geometry appear at each approach.
2. **Given** an intersection, **When** the viewer renders it, **Then** crosswalk markings (striped path surfaces) connect each pair of adjacent corners around the intersection.
3. **Given** multiple intersections exist, **When** the user navigates between them, **Then** each intersection has proportional traffic light placement (no lights placed on small service paths or dead ends).

---

### User Story 3 - Building Colors Are Warm and Distinctive (Priority: P3)

A user sees buildings rendered in warm earth-tone colors (terracotta/brick/sandstone range) that look natural for campus architecture and clearly contrast against the cool grey streets. Building color varies subtly with height so taller structures are visually distinguishable.

**Why this priority**: Clear visual distinction between building and non-building elements is essential for readability of the digital twin. The previous uniform coloring caused buildings to blend with future street and terrain elements.

**Independent Test**: Open viewer and observe that buildings use warm tones (reddish-brown, terracotta, sandstone) while streets use cool greys. Toggle all layers on — each layer type is visually distinct from the others.

**Acceptance Scenarios**:

1. **Given** the viewer with all layers enabled, **When** the user looks at any building, **Then** its color falls within a warm palette (hues between red-orange and yellow-brown) that is clearly different from street grey tones.
2. **Given** buildings of different heights, **When** viewed side by side, **Then** the color varies subtly (taller buildings slightly lighter/brighter, shorter buildings slightly deeper) while staying within the warm palette.
3. **Given** the viewer, **When** the user toggles between color modes, **Then** the "height" mode uses the new warm palette and the "grey" mode remains available as a neutral fallback.

---

### User Story 4 - Street and Intersection Extraction Is a Pipeline Step (Priority: P2)

A developer runs the extraction pipeline and streets with intersections are produced as a step integrated into `build_all.py`. The street/intersection data is stored in the same open binary format as terrain and buildings.

**Why this priority**: Reproducibility is a core project principle (III). Street generation must be automated from LiDAR data, not manually modeled.

**Independent Test**: Run `python tools/build_all.py` and verify street/intersection data files are produced. Run `build_all.py --verify` and confirm streets/in


<!-- LLM output cutoff or reached maximum output limit -->