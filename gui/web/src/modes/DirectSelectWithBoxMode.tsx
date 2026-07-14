import MapboxDraw from '@mapbox/mapbox-gl-draw';
const {
  createSupplementaryPoints,
  constrainFeatureMovement,
  doubleClickZoom,
  moveFeatures
} = MapboxDraw.lib;

const {
  noTarget,
  isOfMetaType,
  isActiveFeature,
  isInactiveFeature,
  isShiftDown,
} = MapboxDraw.lib.CommonSelectors;
const Constants = MapboxDraw.constants;

const isVertex = isOfMetaType(Constants.meta.VERTEX);
const isMidpoint = isOfMetaType(Constants.meta.MIDPOINT);

const DirectSelectWithBoxMode: any = {};

// INTERNAL FUNCTIONS

DirectSelectWithBoxMode.fireUpdate = function () {
  this.map.fire(Constants.events.UPDATE, {
    action: Constants.updateActions.CHANGE_COORDINATES,
    features: this.getSelected().map((f: any) => f.toGeoJSON())
  });
};

DirectSelectWithBoxMode.fireActionable = function (state: any) {
  this.setActionableState({
    combineFeatures: false,
    uncombineFeatures: false,
    trash: state.selectedCoordPaths.length > 0
  });
};

DirectSelectWithBoxMode.startDragging = function (state: any, e: any) {
  this.map.dragPan.disable();
  state.canDragMove = true;
  state.dragMoveLocation = e.lngLat;
};

DirectSelectWithBoxMode.stopDragging = function (state: any) {
  this.map.dragPan.enable();
  state.dragMoving = false;
  state.canDragMove = false;
  state.dragMoveLocation = null;
};

DirectSelectWithBoxMode.onVertex = function (state: any, e: any) {
  this.startDragging(state, e);
  const about = e.featureTarget.properties;
  const selectedIndex = state.selectedCoordPaths.indexOf(about.coord_path);
  if (!isShiftDown(e) && selectedIndex === -1) {
    state.selectedCoordPaths = [about.coord_path];
  } else if (isShiftDown(e) && selectedIndex === -1) {
    state.selectedCoordPaths.push(about.coord_path);
  }

  const selectedCoordinates = this.pathsToCoordinates(state.featureId, state.selectedCoordPaths);
  this.setSelectedCoordinates(selectedCoordinates);
};

DirectSelectWithBoxMode.onMidpoint = function (state: any, e: any) {
  this.startDragging(state, e);
  const about = e.featureTarget.properties;
  state.feature.addCoordinate(about.coord_path, about.lng, about.lat);
  this.fireUpdate();
  state.selectedCoordPaths = [about.coord_path];
};

DirectSelectWithBoxMode.pathsToCoordinates = function (featureId: any, paths: any[]) {
  return paths.map(coord_path => ({ feature_id: featureId, coord_path }));
};

DirectSelectWithBoxMode.onFeature = function (state: any, e: any) {
  // `TouchEvent` is undefined on desktop Safari/Firefox — referencing it bare
  // throws a ReferenceError. Guard with `typeof` before the instanceof check.
  const isTouch = typeof TouchEvent !== 'undefined' && e.originalEvent instanceof TouchEvent;
  // Require Ctrl/Cmd key to drag the entire area on desktop (prevents accidental moves).
  // On touch devices, allow dragging without modifier since there's no keyboard.
  if (state.selectedCoordPaths.length === 0 && (isTouch || e.originalEvent.ctrlKey || e.originalEvent.metaKey)) {
    this.startDragging(state, e);
  } else {
    this.stopDragging(state);
  }
};

DirectSelectWithBoxMode.dragFeature = function (state: any, e: any, delta: any) {
  moveFeatures(this.getSelected(), delta);
  state.dragMoveLocation = e.lngLat;
};

DirectSelectWithBoxMode.dragVertex = function (state: any, delta: any) {
  const selectedCoords = state.selectedCoordPaths.map((coord_path: any) => state.feature.getCoordinate(coord_path));
  const selectedCoordPoints = selectedCoords.map((coords: any) => ({
    type: Constants.geojsonTypes.FEATURE,
    properties: {},
    geometry: {
      type: Constants.geojsonTypes.POINT,
      coordinates: coords
    }
  }));

  const constrainedDelta = constrainFeatureMovement(selectedCoordPoints, delta);
  for (let i = 0; i < selectedCoords.length; i++) {
    const coord = selectedCoords[i];
    state.feature.updateCoordinate(state.selectedCoordPaths[i], coord[0] + constrainedDelta.lng, coord[1] + constrainedDelta.lat);
  }
};

DirectSelectWithBoxMode.clickNoTarget = function () {
  this.changeMode(Constants.modes.SIMPLE_SELECT, {});
};

DirectSelectWithBoxMode.clickInactive = function () {
  this.changeMode(Constants.modes.SIMPLE_SELECT, {});
};

DirectSelectWithBoxMode.clickActiveFeature = function (state: any, e: any) {
  if (e?.originalEvent?.detail >= 2) {
    // Double-click on active feature opens the edit modal
    this.map.fire('feature.open', { feature: state.feature });
    return;
  }
  state.selectedCoordPaths = [];
  this.clearSelectedCoordinates();
  state.feature.changed();
};

// EXTERNAL FUNCTIONS

DirectSelectWithBoxMode.onSetup = function (opts: any) {
  const featureId = opts.featureId;
  const feature = this.getFeature(featureId);

  if (!feature) {
    throw new Error('You must provide a featureId to enter direct_select mode');
  }

  if (feature.type === Constants.geojsonTypes.POINT) {
    throw new TypeError('direct_select mode doesn\'t handle point features');
  }

  const state = {
    featureId,
    feature,
    dragMoveLocation: opts.startPos || null,
    dragMoving: false,
    canDragMove: false,
    selectedCoordPaths: [], //opts.coordPath ? [opts.coordPath] : [],
    boxSelecting: false,
    boxStartLocation: null,
    boxSelectElement: undefined,
  };

  this.setSelectedCoordinates(this.pathsToCoordinates(featureId, state.selectedCoordPaths));
  this.setSelected(featureId);
  doubleClickZoom.disable(this);

  this.setActionableState({
    trash: true
  });

  return state;
};

DirectSelectWithBoxMode.onKeyDown = function (state: any, e: any) {
  if (e.key == "Enter")
    this.map.fire('feature.open', {
      feature: state.feature
    });
};

DirectSelectWithBoxMode.onStop = function () {
  doubleClickZoom.enable(this);
  this.clearSelectedCoordinates();
};

DirectSelectWithBoxMode.toDisplayFeatures = function (state: any, geojson: any, push: any) {
  if (state.featureId === geojson.properties.id) {
    geojson.properties.active = Constants.activeStates.ACTIVE;
    push(geojson);
    createSupplementaryPoints(geojson, {
      midpoints: true,
      selectedPaths: state.selectedCoordPaths
    }).forEach(push);
  } else {
    geojson.properties.active = Constants.activeStates.INACTIVE;
    push(geojson);
  }
  this.fireActionable(state);
};

DirectSelectWithBoxMode.onTrash = function (state: any) {
  state.selectedCoordPaths
    .sort((a: any, b: any) => b.localeCompare(a, 'en', { numeric: true }))
    .forEach((id: any) => state.feature.removeCoordinate(id));
  this.fireUpdate();
  state.selectedCoordPaths = [];
  this.clearSelectedCoordinates();
  this.fireActionable(state);
  if (state.feature.isValid() === false) {
    this.deleteFeature([state.featureId]);
    this.changeMode(Constants.modes.SIMPLE_SELECT, {});
  }
};

DirectSelectWithBoxMode.onMouseMove = function (state: any, e: any) {
  const isFeature = isActiveFeature(e);
  const onVertex = isVertex(e);
  const isMidPoint = isMidpoint(e);
  const noCoords = state.selectedCoordPaths.length === 0;
  // Only show move cursor for whole area when Ctrl/Cmd is held
  if (isFeature && noCoords && (e.originalEvent.ctrlKey || e.originalEvent.metaKey)) this.updateUIClasses({ mouse: Constants.cursors.MOVE });
  else if (isFeature && noCoords) this.updateUIClasses({ mouse: Constants.cursors.POINTER });
  else if (onVertex && !noCoords) this.updateUIClasses({ mouse: Constants.cursors.MOVE });
  else this.updateUIClasses({ mouse: Constants.cursors.NONE });

  const isDraggableItem = onVertex || isFeature || isMidPoint;
  if (isDraggableItem && state.dragMoving) this.fireUpdate();

  return true;
};

DirectSelectWithBoxMode.onMouseOut = function (state: any) {
  if (state.dragMoving) this.fireUpdate();

  return true;
};

DirectSelectWithBoxMode.onMouseDown = DirectSelectWithBoxMode.onTouchStart = function (state: any, e: any) {
  if (isVertex(e)) return this.onVertex(state, e);
  
  if (isShiftDown(e)) {
    this.map.dragPan.disable();
    state.boxSelecting = true;
    state.boxStartLocation = e.point;
    return;
  }

  if (isActiveFeature(e)) return this.onFeature(state, e);
  if (isMidpoint(e)) return this.onMidpoint(state, e);
};

DirectSelectWithBoxMode.onDrag = function (state: any, e: any) {
  if (state.boxSelecting) {
    return this.whileBoxSelect(state, e)
  }

  if (state.canDragMove !== true) return;
  state.dragMoving = true;
  e.originalEvent.stopPropagation();

  const delta = {
    lng: e.lngLat.lng - state.dragMoveLocation.lng,
    lat: e.lngLat.lat - state.dragMoveLocation.lat
  };
  if (state.selectedCoordPaths.length > 0) this.dragVertex(state, delta);
  else this.dragFeature(state, e, delta);

  state.dragMoveLocation = e.lngLat;
};

DirectSelectWithBoxMode.whileBoxSelect = function (state:any, e: any) {
  const boxEndLocation = e.point;
  this.updateUIClasses({ mouse: Constants.cursors.ADD });

  const boxElement = state.boxSelectElement;
  if (!boxElement) {
    state.boxSelectElement = document.createElement('div');
    state.boxSelectElement.classList.add(Constants.classes.BOX_SELECT);
    this.map.getContainer().appendChild(state.boxSelectElement);
  }
  const minX = Math.min(state.boxStartLocation.x, boxEndLocation.x);
  const maxX = Math.max(state.boxStartLocation.x, boxEndLocation.x);
  const minY = Math.min(state.boxStartLocation.y, boxEndLocation.y);
  const maxY = Math.max(state.boxStartLocation.y, boxEndLocation.y);
  const translateValue = `translate(${minX}px, ${minY}px)`;
  state.boxSelectElement.style.transform = translateValue;
  state.boxSelectElement.style.WebkitTransform = translateValue;
  state.boxSelectElement.style.width = `${maxX - minX}px`;
  state.boxSelectElement.style.height = `${maxY - minY}px`;
};

DirectSelectWithBoxMode.onClick = function (state: any, e: any) {
  if (noTarget(e)) return this.clickNoTarget(state, e);
  if (isActiveFeature(e)) return this.clickActiveFeature(state, e);
  if (isInactiveFeature(e)) return this.clickInactive(state, e);
  if (isVertex(e)) {
    if (!isShiftDown(e)) {
      // Clear previous selection if shift is not pressed
      state.selectedCoordPaths = [];
    }
    return this.onVertex(state, e);
  }
  this.stopDragging(state);
};

DirectSelectWithBoxMode.onTap = function (state: any, e: any) {
  if (noTarget(e)) return this.clickNoTarget(state, e);
  if (isActiveFeature(e)) return this.clickActiveFeature(state, e);
  if (isInactiveFeature(e)) return this.clickInactive(state, e);
  if (isVertex(e)) return this.onVertex(state, e);
};

DirectSelectWithBoxMode.onMouseUp = DirectSelectWithBoxMode.onTouchEnd = function (state: any, e: any) {
  if (state.boxSelectElement) {
    this.map.getContainer().removeChild(state.boxSelectElement);
    state.boxSelectElement = undefined;
  }
  if (state.boxSelecting) {
    // End box selection
    state.boxSelecting = false;
    const boxEndLocation = e.point;


    const minX = Math.min(state.boxStartLocation.x, boxEndLocation.x);
    const maxX = Math.max(state.boxStartLocation.x, boxEndLocation.x);
    const minY = Math.min(state.boxStartLocation.y, boxEndLocation.y);
    const maxY = Math.max(state.boxStartLocation.y, boxEndLocation.y);

    const selectedVertices = this.getSelectedVerticesInBox(state.feature, minX, minY, maxX, maxY);


    state.selectedCoordPaths = [...new Set([...state.selectedCoordPaths, ...selectedVertices])];

    this.setSelectedCoordinates(this.pathsToCoordinates(state.featureId, state.selectedCoordPaths));
    state.boxStartLocation = null;

    this.map.dragPan.enable();

    // Trigger a re-render of the feature
    state.feature.changed();

    return;
  }

  if (state.dragMoving) {
    this.fireUpdate();
  }
  this.stopDragging(state);
};


DirectSelectWithBoxMode.getSelectedVerticesInBox = function (feature: any, minX: number, minY: number, maxX: number, maxY: number) {
  const selectedVertices: any[] = [];
  const coordinates = feature.getCoordinates();

  const checkCoordinate = (coord: any, path: any) => {
    const point = this.map.project(coord);
    if (point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY) {
      selectedVertices.push(path);
    }
  };

  const traverseCoordinates = (coords: any, basePath = '') => {
    coords.forEach((coord: any, index: any) => {
      const currentPath = basePath ? `${basePath}.${index}` : `${index}`;
      if (Array.isArray(coord[0])) {
        traverseCoordinates(coord, currentPath);
      } else {
        checkCoordinate(coord, currentPath);
      }
    });
  };

  traverseCoordinates(coordinates);
  return selectedVertices;
};

export default DirectSelectWithBoxMode;
