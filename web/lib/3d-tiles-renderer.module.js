// node_modules/3d-tiles-renderer/src/utilities/urlExtension.js
function getUrlExtension(url) {
  let parsedUrl;
  try {
    parsedUrl = new URL(url, "http://fakehost.com/");
  } catch (_) {
    return null;
  }
  const filename = parsedUrl.pathname.split("/").pop();
  const dotIndex = filename.lastIndexOf(".");
  if (dotIndex === -1 || dotIndex === filename.length - 1) {
    return null;
  }
  const extension = filename.substring(dotIndex + 1);
  return extension;
}

// node_modules/3d-tiles-renderer/src/utilities/LRUCache.js
var GIGABYTE_BYTES = 2 ** 30;
var LRUCache = class {
  get unloadPriorityCallback() {
    return this._unloadPriorityCallback;
  }
  set unloadPriorityCallback(cb) {
    if (cb.length === 1) {
      console.warn('LRUCache: "unloadPriorityCallback" function has been changed to take two arguments.');
      this._unloadPriorityCallback = (a, b) => {
        const valA = cb(a);
        const valB = cb(b);
        if (valA < valB) return -1;
        if (valA > valB) return 1;
        return 0;
      };
    } else {
      this._unloadPriorityCallback = cb;
    }
  }
  constructor() {
    this.minSize = 6e3;
    this.maxSize = 8e3;
    this.minBytesSize = 0.3 * GIGABYTE_BYTES;
    this.maxBytesSize = 0.4 * GIGABYTE_BYTES;
    this.unloadPercent = 0.05;
    this.itemSet = /* @__PURE__ */ new Map();
    this.itemList = [];
    this.usedSet = /* @__PURE__ */ new Set();
    this.callbacks = /* @__PURE__ */ new Map();
    this.markUnusedQueued = false;
    this.unloadingHandle = -1;
    this.cachedBytes = 0;
    this.bytesMap = /* @__PURE__ */ new Map();
    this.loadedSet = /* @__PURE__ */ new Set();
    this._unloadPriorityCallback = null;
    this.computeMemoryUsageCallback = () => null;
    const itemSet = this.itemSet;
    this.defaultPriorityCallback = (item) => itemSet.get(item);
  }
  // Returns whether or not the cache has reached the maximum size
  isFull() {
    return this.itemSet.size >= this.maxSize || this.cachedBytes >= this.maxBytesSize;
  }
  getMemoryUsage(item) {
    return this.bytesMap.get(item) ?? null;
  }
  add(item, removeCb) {
    if (this.markUnusedQueued) {
      this.markAllUnused();
    }
    const itemSet = this.itemSet;
    if (itemSet.has(item)) {
      return false;
    }
    if (this.isFull()) {
      return false;
    }
    const usedSet = this.usedSet;
    const itemList = this.itemList;
    const callbacks = this.callbacks;
    const bytesMap = this.bytesMap;
    itemList.push(item);
    usedSet.add(item);
    itemSet.set(item, Date.now());
    callbacks.set(item, removeCb);
    const bytes = this.computeMemoryUsageCallback(item);
    this.cachedBytes += bytes || 0;
    bytesMap.set(item, bytes);
    return true;
  }
  remove(item) {
    const usedSet = this.usedSet;
    const itemSet = this.itemSet;
    const itemList = this.itemList;
    const bytesMap = this.bytesMap;
    const callbacks = this.callbacks;
    const loadedSet = this.loadedSet;
    if (itemSet.has(item)) {
      this.cachedBytes -= bytesMap.get(item) || 0;
      bytesMap.delete(item);
      callbacks.get(item)(item);
      const index = itemList.indexOf(item);
      itemList.splice(index, 1);
      usedSet.delete(item);
      itemSet.delete(item);
      callbacks.delete(item);
      loadedSet.delete(item);
      return true;
    }
    return false;
  }
  // Marks whether tiles in the cache have been completely loaded or not. Tiles that have not been completely
  // loaded are subject to being disposed early if the cache is full above its max size limits, even if they
  // are marked as used.
  setLoaded(item, value) {
    const { itemSet, loadedSet } = this;
    if (itemSet.has(item)) {
      if (value === true) {
        loadedSet.add(item);
      } else {
        loadedSet.delete(item);
      }
    }
  }
  updateMemoryUsage(item) {
    const itemSet = this.itemSet;
    const bytesMap = this.bytesMap;
    if (!itemSet.has(item)) {
      return;
    }
    this.cachedBytes -= bytesMap.get(item) || 0;
    const bytes = this.computeMemoryUsageCallback(item);
    bytesMap.set(item, bytes);
    this.cachedBytes += bytes;
  }
  markUsed(item) {
    if (this.markUnusedQueued) {
      this.markAllUnused();
    }
    const itemSet = this.itemSet;
    const usedSet = this.usedSet;
    if (itemSet.has(item) && !usedSet.has(item)) {
      itemSet.set(item, Date.now());
      usedSet.add(item);
    }
  }
  markAllUnused() {
    this.usedSet.clear();
    this.markUnusedQueued = false;
    if (this.unloadingHandle !== -1) {
      cancelAnimationFrame(this.unloadingHandle);
      this.unloadingHandle = -1;
    }
  }
  // TODO: this should be renamed because it's not necessarily unloading all unused content
  // Maybe call it "cleanup" or "unloadToMinSize"
  unloadUnusedContent() {
    const {
      unloadPercent,
      minSize,
      maxSize,
      itemList,
      itemSet,
      usedSet,
      loadedSet,
      callbacks,
      bytesMap,
      minBytesSize,
      maxBytesSize
    } = this;
    const unused = itemList.length - usedSet.size;
    const unloaded = itemList.length - loadedSet.size;
    const excessNodes = Math.max(Math.min(itemList.length - minSize, unused), 0);
    const excessBytes = this.cachedBytes - minBytesSize;
    const unloadPriorityCallback = this.unloadPriorityCallback || this.defaultPriorityCallback;
    let needsRerun = false;
    const hasNodesToUnload = excessNodes > 0 && unused > 0 || unloaded && itemList.length > maxSize;
    const hasBytesToUnload = unused && this.cachedBytes > minBytesSize || unloaded && this.cachedBytes > maxBytesSize;
    if (hasBytesToUnload || hasNodesToUnload) {
      itemList.sort((a, b) => {
        const usedA = usedSet.has(a);
        const usedB = usedSet.has(b);
        if (usedA === usedB) {
          const loadedA = loadedSet.has(a);
          const loadedB = loadedSet.has(b);
          if (loadedA === loadedB) {
            return -unloadPriorityCallback(a, b);
          } else {
            return loadedA ? 1 : -1;
          }
        } else {
          return usedA ? 1 : -1;
        }
      });
      const maxUnload = Math.max(minSize * unloadPercent, excessNodes * unloadPercent);
      const nodesToUnload = Math.ceil(Math.min(maxUnload, unused, excessNodes));
      const maxBytesUnload = Math.max(unloadPercent * excessBytes, unloadPercent * minBytesSize);
      const bytesToUnload = Math.min(maxBytesUnload, excessBytes);
      let removedNodes = 0;
      let removedBytes = 0;
      while (this.cachedBytes - removedBytes > maxBytesSize || itemList.length - removedNodes > maxSize) {
        const item = itemList[removedNodes];
        const bytes = bytesMap.get(item) || 0;
        if (usedSet.has(item) && loadedSet.has(item) || this.cachedBytes - removedBytes - bytes < maxBytesSize && itemList.length - removedNodes <= maxSize) {
          break;
        }
        removedBytes += bytes;
        removedNodes++;
      }
      while (removedBytes < bytesToUnload || removedNodes < nodesToUnload) {
        const item = itemList[removedNodes];
        const bytes = bytesMap.get(item) || 0;
        if (usedSet.has(item) || this.cachedBytes - removedBytes - bytes < minBytesSize && removedNodes >= nodesToUnload) {
          break;
        }
        removedBytes += bytes;
        removedNodes++;
      }
      itemList.splice(0, removedNodes).forEach((item) => {
        this.cachedBytes -= bytesMap.get(item) || 0;
        callbacks.get(item)(item);
        bytesMap.delete(item);
        itemSet.delete(item);
        callbacks.delete(item);
        loadedSet.delete(item);
        usedSet.delete(item);
      });
      needsRerun = removedNodes < excessNodes || removedBytes < excessBytes && removedNodes < unused;
      needsRerun = needsRerun && removedNodes > 0;
    }
    if (needsRerun) {
      this.unloadingHandle = requestAnimationFrame(() => this.scheduleUnload());
    }
  }
  scheduleUnload() {
    if (!this.scheduled) {
      this.scheduled = true;
      queueMicrotask(() => {
        this.scheduled = false;
        this.unloadUnusedContent();
        this.markUnusedQueued = true;
      });
    }
  }
};

// node_modules/3d-tiles-renderer/src/utilities/PriorityQueue.js
var PriorityQueue = class {
  constructor() {
    this.maxJobs = 6;
    this.items = [];
    this.callbacks = /* @__PURE__ */ new Map();
    this.currJobs = 0;
    this.scheduled = false;
    this.autoUpdate = true;
    this.priorityCallback = () => {
      throw new Error("PriorityQueue: PriorityCallback function not defined.");
    };
    this.schedulingCallback = (func) => {
      requestAnimationFrame(func);
    };
    this._runjobs = () => {
      this.tryRunJobs();
      this.scheduled = false;
    };
  }
  sort() {
    const priorityCallback2 = this.priorityCallback;
    const items = this.items;
    items.sort(priorityCallback2);
  }
  add(item, callback) {
    return new Promise((resolve, reject) => {
      const prCallback = (...args) => callback(...args).then(resolve).catch(reject);
      const items = this.items;
      const callbacks = this.callbacks;
      items.push(item);
      callbacks.set(item, prCallback);
      if (this.autoUpdate) {
        this.scheduleJobRun();
      }
    });
  }
  remove(item) {
    const items = this.items;
    const callbacks = this.callbacks;
    const index = items.indexOf(item);
    if (index !== -1) {
      items.splice(index, 1);
      callbacks.delete(item);
    }
  }
  tryRunJobs() {
    this.sort();
    const items = this.items;
    const callbacks = this.callbacks;
    const maxJobs = this.maxJobs;
    let currJobs = this.currJobs;
    while (maxJobs > currJobs && items.length > 0) {
      currJobs++;
      const item = items.pop();
      const callback = callbacks.get(item);
      callbacks.delete(item);
      callback(item).then(() => {
        this.currJobs--;
        if (this.autoUpdate) {
          this.scheduleJobRun();
        }
      }).catch(() => {
        this.currJobs--;
        if (this.autoUpdate) {
          this.scheduleJobRun();
        }
      });
    }
    this.currJobs = currJobs;
  }
  scheduleJobRun() {
    if (!this.scheduled) {
      this.schedulingCallback(this._runjobs);
      this.scheduled = true;
    }
  }
};

// node_modules/3d-tiles-renderer/src/base/constants.js
var FAILED = -1;
var UNLOADED = 0;
var LOADING = 1;
var PARSING = 2;
var LOADED = 3;
var WGS84_RADIUS = 6378137;
var WGS84_FLATTENING = 1 / 298.257223563;
var WGS84_HEIGHT = -(WGS84_FLATTENING * WGS84_RADIUS - WGS84_RADIUS);
var LUNAR_HEIGHT = 1736 * 1e3;
var LUNAR_RADIUS = 1738.1 * 1e3;

// node_modules/3d-tiles-renderer/src/base/traverseFunctions.js
function isDownloadFinished(value) {
  return value === LOADED || value === FAILED;
}
function isUsedThisFrame(tile, frameCount) {
  return tile.__lastFrameVisited === frameCount && tile.__used;
}
function resetFrameState(tile, renderer) {
  if (tile.__lastFrameVisited !== renderer.frameCount) {
    tile.__lastFrameVisited = renderer.frameCount;
    tile.__used = false;
    tile.__inFrustum = false;
    tile.__isLeaf = false;
    tile.__visible = false;
    tile.__active = false;
    tile.__error = Infinity;
    tile.__distanceFromCamera = Infinity;
    tile.__childrenWereVisible = false;
    tile.__allChildrenLoaded = false;
    tile.__inFrustum = renderer.tileInView(tile);
    renderer.calculateError(tile);
  }
}
function recursivelyMarkUsed(tile, renderer) {
  renderer.ensureChildrenArePreprocessed(tile);
  resetFrameState(tile, renderer);
  markUsed(tile, renderer);
  if (!tile.__hasRenderableContent) {
    const children = tile.children;
    for (let i = 0, l = children.length; i < l; i++) {
      recursivelyMarkUsed(children[i], renderer);
    }
  }
}
function recursivelyLoadNextRenderableTiles(tile, renderer) {
  renderer.ensureChildrenArePreprocessed(tile);
  const doTraverse = isUsedThisFrame(tile, renderer.frameCount) && !tile.__hasRenderableContent && (!tile.__hasContent || isDownloadFinished(tile.__loadingState));
  if (doTraverse) {
    const children = tile.children;
    for (let i = 0, l = children.length; i < l; i++) {
      const child = children[i];
      recursivelyLoadNextRenderableTiles(child, renderer);
    }
  } else if (isUsedThisFrame(tile, renderer.frameCount) && !renderer.lruCache.isFull()) {
    renderer.queueTileForDownload(tile);
  }
}
function markUsed(tile, renderer) {
  if (tile.__used) {
    return;
  }
  tile.__used = true;
  renderer.lruCache.markUsed(tile);
  renderer.stats.used++;
  if (tile.__inFrustum === true) {
    renderer.stats.inFrustum++;
  }
}
function canTraverse(tile, renderer) {
  if (tile.__error <= renderer.errorTarget) {
    return false;
  }
  if (renderer.maxDepth > 0 && tile.__depth + 1 >= renderer.maxDepth) {
    return false;
  }
  return true;
}
function traverseSet(tile, beforeCb = null, afterCb = null, parent = null, depth = 0) {
  if (beforeCb && beforeCb(tile, parent, depth)) {
    if (afterCb) {
      afterCb(tile, parent, depth);
    }
    return;
  }
  const children = tile.children;
  for (let i = 0, l = children.length; i < l; i++) {
    traverseSet(children[i], beforeCb, afterCb, tile, depth + 1);
  }
  if (afterCb) {
    afterCb(tile, parent, depth);
  }
}
function markUsedTiles(tile, renderer) {
  renderer.ensureChildrenArePreprocessed(tile);
  resetFrameState(tile, renderer);
  if (!tile.__inFrustum) {
    return;
  }
  if (!canTraverse(tile, renderer)) {
    markUsed(tile, renderer);
    return;
  }
  let anyChildrenUsed = false;
  let anyChildrenInFrustum = false;
  const children = tile.children;
  for (let i = 0, l = children.length; i < l; i++) {
    const c = children[i];
    markUsedTiles(c, renderer);
    anyChildrenUsed = anyChildrenUsed || isUsedThisFrame(c, renderer.frameCount);
    anyChildrenInFrustum = anyChildrenInFrustum || c.__inFrustum;
  }
  if (tile.refine === "REPLACE" && !anyChildrenInFrustum && children.length !== 0 && !tile.__hasUnrenderableContent) {
    tile.__inFrustum = false;
    return;
  }
  markUsed(tile, renderer);
  if (anyChildrenUsed && tile.refine === "REPLACE") {
    for (let i = 0, l = children.length; i < l; i++) {
      const c = children[i];
      recursivelyMarkUsed(c, renderer);
    }
  }
}
function markUsedSetLeaves(tile, renderer) {
  const frameCount = renderer.frameCount;
  if (!isUsedThisFrame(tile, frameCount)) {
    return;
  }
  const children = tile.children;
  let anyChildrenUsed = false;
  for (let i = 0, l = children.length; i < l; i++) {
    const c = children[i];
    anyChildrenUsed = anyChildrenUsed || isUsedThisFrame(c, frameCount);
  }
  if (!anyChildrenUsed) {
    tile.__isLeaf = true;
  } else {
    let childrenWereVisible = false;
    let allChildrenLoaded = true;
    for (let i = 0, l = children.length; i < l; i++) {
      const c = children[i];
      markUsedSetLeaves(c, renderer);
      childrenWereVisible = childrenWereVisible || c.__wasSetVisible || c.__childrenWereVisible;
      if (isUsedThisFrame(c, frameCount)) {
        const childLoaded = c.__allChildrenLoaded || c.__hasRenderableContent && isDownloadFinished(c.__loadingState) || !c.__hasContent && c.children.length === 0 || c.__hasUnrenderableContent && c.__loadingState === FAILED;
        allChildrenLoaded = allChildrenLoaded && childLoaded;
      }
    }
    tile.__childrenWereVisible = childrenWereVisible;
    tile.__allChildrenLoaded = allChildrenLoaded;
  }
}
function markVisibleTiles(tile, renderer) {
  const stats = renderer.stats;
  if (!isUsedThisFrame(tile, renderer.frameCount)) {
    return;
  }
  const lruCache = renderer.lruCache;
  if (tile.__isLeaf) {
    if (tile.__loadingState === LOADED) {
      if (tile.__inFrustum) {
        tile.__visible = true;
        stats.visible++;
      }
      tile.__active = true;
      stats.active++;
    } else if (!lruCache.isFull() && tile.__hasContent) {
      renderer.queueTileForDownload(tile);
    }
    return;
  }
  const children = tile.children;
  const hasContent = tile.__hasContent;
  const loadedContent = isDownloadFinished(tile.__loadingState) && hasContent;
  const errorRequirement = (renderer.errorTarget + 1) * renderer.errorThreshold;
  const meetsSSE = tile.__error <= errorRequirement;
  const childrenWereVisible = tile.__childrenWereVisible;
  const allChildrenLoaded = tile.__allChildrenLoaded;
  const includeTile = meetsSSE || tile.refine === "ADD";
  if (includeTile && !loadedContent && !lruCache.isFull() && hasContent) {
    renderer.queueTileForDownload(tile);
  }
  if (meetsSSE && !allChildrenLoaded && !childrenWereVisible && loadedContent || tile.refine === "ADD" && loadedContent) {
    if (tile.__inFrustum) {
      tile.__visible = true;
      stats.visible++;
    }
    tile.__active = true;
    stats.active++;
  }
  if (tile.refine === "REPLACE" && meetsSSE && !allChildrenLoaded) {
    for (let i = 0, l = children.length; i < l; i++) {
      const c = children[i];
      if (isUsedThisFrame(c, renderer.frameCount)) {
        recursivelyLoadNextRenderableTiles(c, renderer);
      }
    }
  } else {
    for (let i = 0, l = children.length; i < l; i++) {
      markVisibleTiles(children[i], renderer);
    }
  }
}
function toggleTiles(tile, renderer) {
  const isUsed = isUsedThisFrame(tile, renderer.frameCount);
  if (isUsed || tile.__usedLastFrame) {
    let setActive = false;
    let setVisible = false;
    if (isUsed) {
      setActive = tile.__active;
      if (renderer.displayActiveTiles) {
        setVisible = tile.__active || tile.__visible;
      } else {
        setVisible = tile.__visible;
      }
    } else {
      resetFrameState(tile, renderer);
    }
    if (tile.__hasRenderableContent && tile.__loadingState === LOADED) {
      if (tile.__wasSetActive !== setActive) {
        renderer.setTileActive(tile, setActive);
      }
      if (tile.__wasSetVisible !== setVisible) {
        renderer.setTileVisible(tile, setVisible);
      }
    }
    tile.__wasSetActive = setActive;
    tile.__wasSetVisible = setVisible;
    tile.__usedLastFrame = isUsed;
    const children = tile.children;
    for (let i = 0, l = children.length; i < l; i++) {
      const c = children[i];
      toggleTiles(c, renderer);
    }
  }
}

// node_modules/3d-tiles-renderer/src/base/TilesRendererBase.js
var PLUGIN_REGISTERED = /* @__PURE__ */ Symbol("PLUGIN_REGISTERED");
var priorityCallback = (a, b) => {
  if (a.__depthFromRenderedParent !== b.__depthFromRenderedParent) {
    return a.__depthFromRenderedParent > b.__depthFromRenderedParent ? -1 : 1;
  } else if (a.__inFrustum !== b.__inFrustum) {
    return a.__inFrustum ? 1 : -1;
  } else if (a.__used !== b.__used) {
    return a.__used ? 1 : -1;
  } else if (a.__error !== b.__error) {
    return a.__error > b.__error ? 1 : -1;
  } else if (a.__distanceFromCamera !== b.__distanceFromCamera) {
    return a.__distanceFromCamera > b.__distanceFromCamera ? -1 : 1;
  }
  return 0;
};
var lruPriorityCallback = (a, b) => {
  if (a.__depthFromRenderedParent !== b.__depthFromRenderedParent) {
    return a.__depthFromRenderedParent > b.__depthFromRenderedParent ? 1 : -1;
  } else if (a.__loadingState !== b.__loadingState) {
    return a.__loadingState > b.__loadingState ? -1 : 1;
  } else if (a.__lastFrameVisited !== b.__lastFrameVisited) {
    return a.__lastFrameVisited > b.__lastFrameVisited ? -1 : 1;
  } else if (a.__hasUnrenderableContent !== b.__hasUnrenderableContent) {
    return a.__hasUnrenderableContent ? -1 : 1;
  } else if (a.__error !== b.__error) {
    return a.__error > b.__error ? -1 : 1;
  }
  return 0;
};
var TilesRendererBase = class {
  get root() {
    const tileSet = this.rootTileSet;
    return tileSet ? tileSet.root : null;
  }
  set loadSiblings(v) {
    console.warn('TilesRenderer: "loadSiblings" option has been removed.');
  }
  set stopAtEmptyTiles(v) {
    console.warn('TilesRenderer: "stopAtEmptyTiles" option has been removed.');
  }
  set preprocessURL(v) {
    console.warn('TilesRendererBase: The "preprocessURL" callback has been deprecated. Use a plugin, instead.');
    this._preprocessURL = v;
  }
  get preprocessURL() {
    return this._preprocessURL;
  }
  constructor(url = null) {
    this.rootLoadingState = UNLOADED;
    this.rootTileSet = null;
    this.rootURL = url;
    this.fetchOptions = {};
    this.plugins = [];
    this.queuedTiles = [];
    this._preprocessURL = null;
    const lruCache = new LRUCache();
    lruCache.unloadPriorityCallback = lruPriorityCallback;
    const downloadQueue = new PriorityQueue();
    downloadQueue.maxJobs = 10;
    downloadQueue.priorityCallback = priorityCallback;
    const parseQueue = new PriorityQueue();
    parseQueue.maxJobs = 1;
    parseQueue.priorityCallback = priorityCallback;
    this.lruCache = lruCache;
    this.downloadQueue = downloadQueue;
    this.parseQueue = parseQueue;
    this.stats = {
      parsing: 0,
      downloading: 0,
      failed: 0,
      inFrustum: 0,
      used: 0,
      active: 0,
      visible: 0
    };
    this.frameCount = 0;
    this.errorTarget = 6;
    this.errorThreshold = Infinity;
    this.displayActiveTiles = false;
    this.maxDepth = Infinity;
  }
  // Plugins
  registerPlugin(plugin) {
    if (plugin[PLUGIN_REGISTERED] === true) {
      throw new Error("TilesRendererBase: A plugin can only be registered to a single tile set");
    }
    this.plugins.push(plugin);
    plugin[PLUGIN_REGISTERED] = true;
    if (plugin.init) {
      plugin.init(this);
    }
  }
  unregisterPlugin(plugin) {
    const plugins = this.plugins;
    if (typeof plugin === "string") {
      plugin = this.getPluginByName(name);
    }
    if (plugins.includes(plugin)) {
      const index = plugins.indexOf(plugin);
      plugins.splice(index, 1);
      if (plugin.dispose) {
        plugin.dispose();
      }
      return true;
    }
    return false;
  }
  getPluginByName(name2) {
    return this.plugins.find((p) => p.name === name2) || null;
  }
  traverse(beforecb, aftercb) {
    if (!this.root) return;
    traverseSet(this.root, (tile, ...args) => {
      this.ensureChildrenArePreprocessed(tile);
      return beforecb ? beforecb(tile, ...args) : false;
    }, aftercb);
  }
  queueTileForDownload(tile) {
    this.queuedTiles.push(tile);
  }
  // Public API
  update() {
    const stats = this.stats;
    const lruCache = this.lruCache;
    if (this.rootLoadingState === UNLOADED) {
      this.rootLoadingState = LOADING;
      this.invokeOnePlugin((plugin) => plugin.loadRootTileSet && plugin.loadRootTileSet()).then(() => this.rootLoadingState = LOADED).catch(() => this.rootLoadingState = FAILED);
    }
    if (!this.root) {
      return;
    }
    const root = this.root;
    stats.inFrustum = 0;
    stats.used = 0;
    stats.active = 0;
    stats.visible = 0;
    this.frameCount++;
    markUsedTiles(root, this);
    markUsedSetLeaves(root, this);
    markVisibleTiles(root, this);
    toggleTiles(root, this);
    const queuedTiles = this.queuedTiles;
    queuedTiles.sort(lruCache.unloadPriorityCallback);
    for (let i = 0, l = queuedTiles.length; i < l && !lruCache.isFull(); i++) {
      this.requestTileContents(queuedTiles[i]);
    }
    queuedTiles.length = 0;
    lruCache.scheduleUnload();
  }
  resetFailedTiles() {
    if (this.rootLoadingState === FAILED) {
      this.rootLoadingState = UNLOADED;
    }
    const stats = this.stats;
    if (stats.failed === 0) {
      return;
    }
    this.traverse((tile) => {
      if (tile.__loadingState === FAILED) {
        tile.__loadingState = UNLOADED;
      }
    });
    stats.failed = 0;
  }
  dispose() {
    this.invokeAllPlugins((plugin) => {
      plugin !== this && plugin.dispose && plugin.dispose();
    });
    const lruCache = this.lruCache;
    const toRemove = [];
    this.traverse((t) => {
      toRemove.push(t);
      return false;
    });
    for (let i = 0, l = toRemove.length; i < l; i++) {
      lruCache.remove(toRemove[i]);
    }
    this.stats = {
      parsing: 0,
      downloading: 0,
      failed: 0,
      inFrustum: 0,
      used: 0,
      active: 0,
      visible: 0
    };
    this.frameCount = 0;
  }
  // Overrideable
  fetchData(url, options) {
    return fetch(url, options);
  }
  parseTile(buffer, tile, extension) {
    return null;
  }
  disposeTile(tile) {
    if (tile.__visible) {
      this.setTileVisible(tile, false);
      tile.__visible = false;
    }
    if (tile.__active) {
      this.setTileActive(tile, false);
      tile.__active = false;
    }
  }
  preprocessNode(tile, tileSetDir, parentTile = null) {
    if (tile.content) {
      if (!("uri" in tile.content) && "url" in tile.content) {
        tile.content.uri = tile.content.url;
        delete tile.content.url;
      }
      if (tile.content.boundingVolume && !("box" in tile.content.boundingVolume || "sphere" in tile.content.boundingVolume || "region" in tile.content.boundingVolume)) {
        delete tile.content.boundingVolume;
      }
    }
    tile.parent = parentTile;
    tile.children = tile.children || [];
    if (tile.content?.uri) {
      const extension = getUrlExtension(tile.content.uri);
      tile.__hasContent = true;
      tile.__hasUnrenderableContent = Boolean(extension && /json$/.test(extension));
      tile.__hasRenderableContent = !tile.__hasUnrenderableContent;
    } else {
      tile.__hasContent = false;
      tile.__hasUnrenderableContent = false;
      tile.__hasRenderableContent = false;
    }
    tile.__distanceFromCamera = Infinity;
    tile.__error = Infinity;
    tile.__inFrustum = false;
    tile.__isLeaf = false;
    tile.__usedLastFrame = false;
    tile.__used = false;
    tile.__wasSetVisible = false;
    tile.__visible = false;
    tile.__childrenWereVisible = false;
    tile.__allChildrenLoaded = false;
    tile.__wasSetActive = false;
    tile.__active = false;
    tile.__loadingState = UNLOADED;
    tile.__loadIndex = 0;
    tile.__loadAbort = null;
    if (parentTile === null) {
      tile.__depth = 0;
      tile.__depthFromRenderedParent = 0;
      tile.refine = tile.refine || "REPLACE";
    } else {
      tile.__depth = parentTile.__depth + 1;
      tile.__depthFromRenderedParent = parentTile.__depthFromRenderedParent + (tile.__hasRenderableContent ? 1 : 0);
      tile.refine = tile.refine || parentTile.refine;
    }
    tile.__basePath = tileSetDir;
    tile.__lastFrameVisited = -1;
    this.invokeAllPlugins((plugin) => {
      plugin !== this && plugin.preprocessNode && plugin.preprocessNode(tile, tileSetDir, parentTile);
    });
  }
  setTileActive(tile, state) {
  }
  setTileVisible(tile, state) {
  }
  calculateError(tile) {
    return 0;
  }
  tileInView(tile) {
    return true;
  }
  ensureChildrenArePreprocessed(tile) {
    const children = tile.children;
    for (let i = 0, l = children.length; i < l; i++) {
      const child = children[i];
      if ("__depth" in child) {
        break;
      }
      this.preprocessNode(child, tile.__basePath, tile);
    }
  }
  // Private Functions
  preprocessTileSet(json, url, parent = null) {
    const version = json.asset.version;
    const [major, minor] = version.split(".").map((v) => parseInt(v));
    console.assert(
      major <= 1,
      "TilesRenderer: asset.version is expected to be a 1.x or a compatible version."
    );
    if (major === 1 && minor > 0) {
      console.warn("TilesRenderer: tiles versions at 1.1 or higher have limited support. Some new extensions and features may not be supported.");
    }
    let basePath = url.replace(/\/[^/]*\/?$/, "");
    basePath = new URL(basePath, window.location.href).toString();
    this.preprocessNode(json.root, basePath, parent);
  }
  loadRootTileSet() {
    let processedUrl = this.rootURL;
    this.invokeAllPlugins((plugin) => processedUrl = plugin.preprocessURL ? plugin.preprocessURL(processedUrl, null) : processedUrl);
    const pr = this.invokeOnePlugin((plugin) => plugin.fetchData && plugin.fetchData(processedUrl, this.fetchOptions)).then((res) => {
      if (res.ok) {
        return res.json();
      } else {
        throw new Error(`TilesRenderer: Failed to load tileset "${processedUrl}" with status ${res.status} : ${res.statusText}`);
      }
    }).then((json) => {
      this.preprocessTileSet(json, processedUrl);
      this.rootTileSet = json;
    });
    pr.catch((err) => {
      console.error(err);
      this.rootTileSet = null;
    });
    return pr;
  }
  requestTileContents(tile) {
    if (tile.__loadingState !== UNLOADED) {
      return;
    }
    let isExternalTileSet = false;
    let uri = new URL(tile.content.uri, tile.__basePath + "/").toString();
    this.invokeAllPlugins((plugin) => uri = plugin.preprocessURL ? plugin.preprocessURL(uri, tile) : uri);
    const stats = this.stats;
    const lruCache = this.lruCache;
    const downloadQueue = this.downloadQueue;
    const parseQueue = this.parseQueue;
    const extension = getUrlExtension(uri);
    const addedSuccessfully = lruCache.add(tile, (t) => {
      if (t.__loadingState === LOADING) {
        t.__loadAbort.abort();
        t.__loadAbort = null;
      } else if (isExternalTileSet) {
        t.children.length = 0;
      } else {
        this.invokeAllPlugins((plugin) => {
          plugin.disposeTile && plugin.disposeTile(t);
        });
      }
      if (t.__loadingState === LOADING) {
        stats.downloading--;
      } else if (t.__loadingState === PARSING) {
        stats.parsing--;
      }
      t.__loadingState = UNLOADED;
      t.__loadIndex++;
      parseQueue.remove(t);
      downloadQueue.remove(t);
    });
    if (!addedSuccessfully) {
      return;
    }
    tile.__loadIndex++;
    const loadIndex = tile.__loadIndex;
    const controller = new AbortController();
    const signal = controller.signal;
    stats.downloading++;
    tile.__loadAbort = controller;
    tile.__loadingState = LOADING;
    const errorCallback = (e) => {
      if (tile.__loadIndex !== loadIndex) {
        return;
      }
      if (e.name !== "AbortError") {
        parseQueue.remove(tile);
        downloadQueue.remove(tile);
        if (tile.__loadingState === PARSING) {
          stats.parsing--;
        } else if (tile.__loadingState === LOADING) {
          stats.downloading--;
        }
        stats.failed++;
        console.error(`TilesRenderer : Failed to load tile at url "${tile.content.uri}".`);
        console.error(e);
        tile.__loadingState = FAILED;
        lruCache.setLoaded(tile, true);
      } else {
        lruCache.remove(tile);
      }
    };
    return downloadQueue.add(tile, (downloadTile) => {
      if (downloadTile.__loadIndex !== loadIndex) {
        return Promise.resolve();
      }
      return this.invokeOnePlugin((plugin) => plugin.fetchData && plugin.fetchData(uri, { ...this.fetchOptions, signal }));
    }).then((res) => {
      if (tile.__loadIndex !== loadIndex) {
        return;
      }
      if (res.ok) {
        return extension === "json" ? res.json() : res.arrayBuffer();
      } else {
        throw new Error(`Failed to load model with error code ${res.status}`);
      }
    }).then((content) => {
      if (tile.__loadIndex !== loadIndex) {
        return;
      }
      stats.downloading--;
      stats.parsing++;
      tile.__loadAbort = null;
      tile.__loadingState = PARSING;
      return parseQueue.add(tile, (parseTile) => {
        if (parseTile.__loadIndex !== loadIndex) {
          return Promise.resolve();
        }
        if (extension === "json" && content.root) {
          this.preprocessTileSet(content, uri, tile);
          tile.children.push(content.root);
          isExternalTileSet = true;
          return Promise.resolve();
        } else {
          return this.invokeOnePlugin((plugin) => plugin.parseTile && plugin.parseTile(content, parseTile, extension, uri));
        }
      });
    }).then(() => {
      if (tile.__loadIndex !== loadIndex) {
        return;
      }
      stats.parsing--;
      tile.__loadingState = LOADED;
      lruCache.setLoaded(tile, true);
      if (lruCache.getMemoryUsage(tile) === null) {
        if (lruCache.isFull() && lruCache.computeMemoryUsageCallback(tile) > 0) {
          lruCache.remove(tile);
        } else {
          lruCache.updateMemoryUsage(tile);
        }
      }
    }).catch(errorCallback);
  }
  getAttributions(target = []) {
    this.invokeAllPlugins((plugin) => plugin !== this && plugin.getAttributions && plugin.getAttributions(target));
    return target;
  }
  invokeOnePlugin(func) {
    const plugins = [...this.plugins, this];
    for (let i = 0; i < plugins.length; i++) {
      const result = func(plugins[i]);
      if (result) {
        return result;
      }
    }
    return null;
  }
  invokeAllPlugins(func) {
    const plugins = [...this.plugins, this];
    const pending = [];
    for (let i = 0; i < plugins.length; i++) {
      const result = func(plugins[i]);
      if (result) {
        pending.push(result);
      }
    }
    return pending.length === 0 ? null : Promise.all(pending);
  }
};

// node_modules/3d-tiles-renderer/src/utilities/arrayToString.js
var utf8decoder = new TextDecoder();
function arrayToString(array) {
  return utf8decoder.decode(array);
}

// node_modules/3d-tiles-renderer/src/utilities/FeatureTable.js
function parseBinArray(buffer, arrayStart, count, type, componentType, propertyName) {
  let stride;
  switch (type) {
    case "SCALAR":
      stride = 1;
      break;
    case "VEC2":
      stride = 2;
      break;
    case "VEC3":
      stride = 3;
      break;
    case "VEC4":
      stride = 4;
      break;
    default:
      throw new Error(`FeatureTable : Feature type not provided for "${propertyName}".`);
  }
  let data;
  const arrayLength = count * stride;
  switch (componentType) {
    case "BYTE":
      data = new Int8Array(buffer, arrayStart, arrayLength);
      break;
    case "UNSIGNED_BYTE":
      data = new Uint8Array(buffer, arrayStart, arrayLength);
      break;
    case "SHORT":
      data = new Int16Array(buffer, arrayStart, arrayLength);
      break;
    case "UNSIGNED_SHORT":
      data = new Uint16Array(buffer, arrayStart, arrayLength);
      break;
    case "INT":
      data = new Int32Array(buffer, arrayStart, arrayLength);
      break;
    case "UNSIGNED_INT":
      data = new Uint32Array(buffer, arrayStart, arrayLength);
      break;
    case "FLOAT":
      data = new Float32Array(buffer, arrayStart, arrayLength);
      break;
    case "DOUBLE":
      data = new Float64Array(buffer, arrayStart, arrayLength);
      break;
    default:
      throw new Error(`FeatureTable : Feature component type not provided for "${propertyName}".`);
  }
  return data;
}
var FeatureTable = class {
  constructor(buffer, start, headerLength, binLength) {
    this.buffer = buffer;
    this.binOffset = start + headerLength;
    this.binLength = binLength;
    let header = null;
    if (headerLength !== 0) {
      const headerData = new Uint8Array(buffer, start, headerLength);
      header = JSON.parse(arrayToString(headerData));
    } else {
      header = {};
    }
    this.header = header;
  }
  getKeys() {
    return Object.keys(this.header);
  }
  getData(key, count, defaultComponentType = null, defaultType = null) {
    const header = this.header;
    if (!(key in header)) {
      return null;
    }
    const feature = header[key];
    if (!(feature instanceof Object)) {
      return feature;
    } else if (Array.isArray(feature)) {
      return feature;
    } else {
      const { buffer, binOffset, binLength } = this;
      const byteOffset = feature.byteOffset || 0;
      const featureType = feature.type || defaultType;
      const featureComponentType = feature.componentType || defaultComponentType;
      if ("type" in feature && defaultType && feature.type !== defaultType) {
        throw new Error("FeatureTable: Specified type does not match expected type.");
      }
      const arrayStart = binOffset + byteOffset;
      const data = parseBinArray(buffer, arrayStart, count, featureType, featureComponentType, key);
      const dataEnd = arrayStart + data.byteLength;
      if (dataEnd > binOffset + binLength) {
        throw new Error("FeatureTable: Feature data read outside binary body length.");
      }
      return data;
    }
  }
  getBuffer(byteOffset, byteLength) {
    const { buffer, binOffset } = this;
    return buffer.slice(binOffset + byteOffset, binOffset + byteOffset + byteLength);
  }
};

// node_modules/3d-tiles-renderer/src/utilities/BatchTableHierarchyExtension.js
var BatchTableHierarchyExtension = class {
  constructor(batchTable) {
    this.batchTable = batchTable;
    const extensionHeader = batchTable.header.extensions["3DTILES_batch_table_hierarchy"];
    this.classes = extensionHeader.classes;
    for (const classDef of this.classes) {
      const instances = classDef.instances;
      for (const property in instances) {
        classDef.instances[property] = this._parseProperty(instances[property], classDef.length, property);
      }
    }
    this.instancesLength = extensionHeader.instancesLength;
    this.classIds = this._parseProperty(extensionHeader.classIds, this.instancesLength, "classIds");
    if (extensionHeader.parentCounts) {
      this.parentCounts = this._parseProperty(extensionHeader.parentCounts, this.instancesLength, "parentCounts");
    } else {
      this.parentCounts = new Array(this.instancesLength).fill(1);
    }
    if (extensionHeader.parentIds) {
      const parentIdsLength = this.parentCounts.reduce((a, b) => a + b, 0);
      this.parentIds = this._parseProperty(extensionHeader.parentIds, parentIdsLength, "parentIds");
    } else {
      this.parentIds = null;
    }
    this.instancesIds = [];
    const classCounter = {};
    for (const classId of this.classIds) {
      classCounter[classId] = classCounter[classId] ?? 0;
      this.instancesIds.push(classCounter[classId]);
      classCounter[classId]++;
    }
  }
  _parseProperty(property, propertyLength, propertyName) {
    if (Array.isArray(property)) {
      return property;
    } else {
      const { buffer, binOffset } = this.batchTable;
      const byteOffset = property.byteOffset;
      const componentType = property.componentType || "UNSIGNED_SHORT";
      const arrayStart = binOffset + byteOffset;
      return parseBinArray(buffer, arrayStart, propertyLength, "SCALAR", componentType, propertyName);
    }
  }
  getDataFromId(id, target = {}) {
    const parentCount = this.parentCounts[id];
    if (this.parentIds && parentCount > 0) {
      let parentIdsOffset = 0;
      for (let i = 0; i < id; i++) {
        parentIdsOffset += this.parentCounts[i];
      }
      for (let i = 0; i < parentCount; i++) {
        const parentId = this.parentIds[parentIdsOffset + i];
        if (parentId !== id) {
          this.getDataFromId(parentId, target);
        }
      }
    }
    const classId = this.classIds[id];
    const instances = this.classes[classId].instances;
    const className = this.classes[classId].name;
    const instanceId = this.instancesIds[id];
    for (const key in instances) {
      target[className] = target[className] || {};
      target[className][key] = instances[key][instanceId];
    }
    return target;
  }
};

// node_modules/3d-tiles-renderer/src/utilities/BatchTable.js
var BatchTable = class extends FeatureTable {
  get batchSize() {
    console.warn("BatchTable.batchSize has been deprecated and replaced with BatchTable.count.");
    return this.count;
  }
  constructor(buffer, count, start, headerLength, binLength) {
    super(buffer, start, headerLength, binLength);
    this.count = count;
    this.extensions = {};
    const extensions = this.header.extensions;
    if (extensions) {
      if (extensions["3DTILES_batch_table_hierarchy"]) {
        this.extensions["3DTILES_batch_table_hierarchy"] = new BatchTableHierarchyExtension(this);
      }
    }
  }
  getData(key, componentType = null, type = null) {
    console.warn("BatchTable: BatchTable.getData is deprecated. Use BatchTable.getDataFromId to get allproperties for an id or BatchTable.getPropertyArray for getting an array of value for a property.");
    return super.getData(key, this.count, componentType, type);
  }
  getDataFromId(id, target = {}) {
    if (id < 0 || id >= this.count) {
      throw new Error(`BatchTable: id value "${id}" out of bounds for "${this.count}" features number.`);
    }
    for (const key of this.getKeys()) {
      if (key !== "extensions") {
        target[key] = super.getData(key, this.count)[id];
      }
    }
    for (const extensionName in this.extensions) {
      const extension = this.extensions[extensionName];
      if (extension.getDataFromId instanceof Function) {
        target[extensionName] = target[extensionName] || {};
        extension.getDataFromId(id, target[extensionName]);
      }
    }
    return target;
  }
  getPropertyArray(key) {
    return super.getData(key, this.count);
  }
};

// node_modules/3d-tiles-renderer/src/base/loaders/LoaderBase.js
var LoaderBase = class {
  constructor() {
    this.fetchOptions = {};
    this.workingPath = "";
  }
  load(url) {
    console.warn('Loader: "load" function has been deprecated in favor of "loadAsync".');
    return this.loadAsync(url);
  }
  loadAsync(url) {
    return fetch(url, this.fetchOptions).then((res) => {
      if (!res.ok) {
        throw new Error(`Failed to load file "${url}" with status ${res.status} : ${res.statusText}`);
      }
      return res.arrayBuffer();
    }).then((buffer) => {
      if (this.workingPath === "") {
        this.workingPath = this.workingPathForURL(url);
      }
      return this.parse(buffer);
    });
  }
  resolveExternalURL(url) {
    if (/^[^\\/]/.test(url) && !/^http/.test(url)) {
      return this.workingPath + "/" + url;
    } else {
      return url;
    }
  }
  workingPathForURL(url) {
    const splits = url.split(/[\\/]/g);
    splits.pop();
    const workingPath = splits.join("/");
    return workingPath + "/";
  }
  parse(buffer) {
    throw new Error("LoaderBase: Parse not implemented.");
  }
};

// node_modules/3d-tiles-renderer/src/utilities/readMagicBytes.js
function readMagicBytes(bufferOrDataView) {
  let view;
  if (bufferOrDataView instanceof DataView) {
    view = bufferOrDataView;
  } else {
    view = new DataView(bufferOrDataView);
  }
  if (String.fromCharCode(view.getUint8(0)) === "{") {
    return null;
  }
  let magicBytes = "";
  for (let i = 0; i < 4; i++) {
    magicBytes += String.fromCharCode(view.getUint8(i));
  }
  return magicBytes;
}

// node_modules/3d-tiles-renderer/src/base/loaders/B3DMLoaderBase.js
var B3DMLoaderBase = class extends LoaderBase {
  parse(buffer) {
    const dataView = new DataView(buffer);
    const magic = readMagicBytes(dataView);
    console.assert(magic === "b3dm");
    const version = dataView.getUint32(4, true);
    console.assert(version === 1);
    const byteLength = dataView.getUint32(8, true);
    console.assert(byteLength === buffer.byteLength);
    const featureTableJSONByteLength = dataView.getUint32(12, true);
    const featureTableBinaryByteLength = dataView.getUint32(16, true);
    const batchTableJSONByteLength = dataView.getUint32(20, true);
    const batchTableBinaryByteLength = dataView.getUint32(24, true);
    const featureTableStart = 28;
    const featureTableBuffer = buffer.slice(
      featureTableStart,
      featureTableStart + featureTableJSONByteLength + featureTableBinaryByteLength
    );
    const featureTable = new FeatureTable(
      featureTableBuffer,
      0,
      featureTableJSONByteLength,
      featureTableBinaryByteLength
    );
    const batchTableStart = featureTableStart + featureTableJSONByteLength + featureTableBinaryByteLength;
    const batchTableBuffer = buffer.slice(
      batchTableStart,
      batchTableStart + batchTableJSONByteLength + batchTableBinaryByteLength
    );
    const batchTable = new BatchTable(
      batchTableBuffer,
      featureTable.getData("BATCH_LENGTH"),
      0,
      batchTableJSONByteLength,
      batchTableBinaryByteLength
    );
    const glbStart = batchTableStart + batchTableJSONByteLength + batchTableBinaryByteLength;
    const glbBytes = new Uint8Array(buffer, glbStart, byteLength - glbStart);
    return {
      version,
      featureTable,
      batchTable,
      glbBytes
    };
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/B3DMLoader.js
import { DefaultLoadingManager, Matrix4 as Matrix42 } from "three";

// node_modules/three/examples/jsm/loaders/GLTFLoader.js
import {
  AnimationClip,
  Bone,
  Box3,
  BufferAttribute as BufferAttribute2,
  BufferGeometry as BufferGeometry2,
  ClampToEdgeWrapping,
  Color,
  ColorManagement,
  DirectionalLight,
  DoubleSide,
  FileLoader,
  FrontSide,
  Group,
  ImageBitmapLoader,
  InstancedMesh,
  InterleavedBuffer as InterleavedBuffer2,
  InterleavedBufferAttribute as InterleavedBufferAttribute2,
  Interpolant,
  InterpolateDiscrete,
  InterpolateLinear,
  Line,
  LineBasicMaterial,
  LineLoop,
  LineSegments,
  LinearFilter,
  LinearMipmapLinearFilter,
  LinearMipmapNearestFilter,
  LinearSRGBColorSpace,
  Loader,
  LoaderUtils,
  Material,
  MathUtils,
  Matrix4,
  Mesh,
  MeshBasicMaterial,
  MeshPhysicalMaterial,
  MeshStandardMaterial,
  MirroredRepeatWrapping,
  NearestFilter,
  NearestMipmapLinearFilter,
  NearestMipmapNearestFilter,
  NumberKeyframeTrack,
  Object3D,
  OrthographicCamera,
  PerspectiveCamera,
  PointLight,
  Points,
  PointsMaterial,
  PropertyBinding,
  Quaternion,
  QuaternionKeyframeTrack,
  RepeatWrapping,
  Skeleton,
  SkinnedMesh,
  Sphere,
  SpotLight,
  Texture,
  TextureLoader,
  TriangleFanDrawMode as TriangleFanDrawMode2,
  TriangleStripDrawMode as TriangleStripDrawMode2,
  Vector2,
  Vector3 as Vector32,
  VectorKeyframeTrack,
  SRGBColorSpace,
  InstancedBufferAttribute as InstancedBufferAttribute2
} from "three";

// node_modules/three/examples/jsm/utils/BufferGeometryUtils.js
import {
  BufferAttribute,
  BufferGeometry,
  Float32BufferAttribute,
  InstancedBufferAttribute,
  InterleavedBuffer,
  InterleavedBufferAttribute,
  TriangleFanDrawMode,
  TriangleStripDrawMode,
  TrianglesDrawMode,
  Vector3
} from "three";
function estimateBytesUsed(geometry) {
  let mem = 0;
  for (const name2 in geometry.attributes) {
    const attr = geometry.getAttribute(name2);
    mem += attr.count * attr.itemSize * attr.array.BYTES_PER_ELEMENT;
  }
  const indices = geometry.getIndex();
  mem += indices ? indices.count * indices.itemSize * indices.array.BYTES_PER_ELEMENT : 0;
  return mem;
}
function toTrianglesDrawMode(geometry, drawMode) {
  if (drawMode === TrianglesDrawMode) {
    console.warn("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Geometry already defined as triangles.");
    return geometry;
  }
  if (drawMode === TriangleFanDrawMode || drawMode === TriangleStripDrawMode) {
    let index = geometry.getIndex();
    if (index === null) {
      const indices = [];
      const position = geometry.getAttribute("position");
      if (position !== void 0) {
        for (let i = 0; i < position.count; i++) {
          indices.push(i);
        }
        geometry.setIndex(indices);
        index = geometry.getIndex();
      } else {
        console.error("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Undefined position attribute. Processing not possible.");
        return geometry;
      }
    }
    const numberOfTriangles = index.count - 2;
    const newIndices = [];
    if (drawMode === TriangleFanDrawMode) {
      for (let i = 1; i <= numberOfTriangles; i++) {
        newIndices.push(index.getX(0));
        newIndices.push(index.getX(i));
        newIndices.push(index.getX(i + 1));
      }
    } else {
      for (let i = 0; i < numberOfTriangles; i++) {
        if (i % 2 === 0) {
          newIndices.push(index.getX(i));
          newIndices.push(index.getX(i + 1));
          newIndices.push(index.getX(i + 2));
        } else {
          newIndices.push(index.getX(i + 2));
          newIndices.push(index.getX(i + 1));
          newIndices.push(index.getX(i));
        }
      }
    }
    if (newIndices.length / 3 !== numberOfTriangles) {
      console.error("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Unable to generate correct amount of triangles.");
    }
    const newGeometry = geometry.clone();
    newGeometry.setIndex(newIndices);
    newGeometry.clearGroups();
    return newGeometry;
  } else {
    console.error("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Unknown draw mode:", drawMode);
    return geometry;
  }
}

// node_modules/three/examples/jsm/loaders/GLTFLoader.js
var GLTFLoader = class extends Loader {
  constructor(manager) {
    super(manager);
    this.dracoLoader = null;
    this.ktx2Loader = null;
    this.meshoptDecoder = null;
    this.pluginCallbacks = [];
    this.register(function(parser) {
      return new GLTFMaterialsClearcoatExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFTextureBasisUExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFTextureWebPExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFTextureAVIFExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsSheenExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsTransmissionExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsVolumeExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsIorExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsEmissiveStrengthExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsSpecularExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsIridescenceExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsAnisotropyExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMaterialsBumpExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFLightsExtension(parser);
    });
    this.register(function(parser) {
      return new GLTFMeshoptCompression(parser);
    });
    this.register(function(parser) {
      return new GLTFMeshGpuInstancing(parser);
    });
  }
  load(url, onLoad, onProgress, onError) {
    const scope = this;
    let resourcePath;
    if (this.resourcePath !== "") {
      resourcePath = this.resourcePath;
    } else if (this.path !== "") {
      const relativeUrl = LoaderUtils.extractUrlBase(url);
      resourcePath = LoaderUtils.resolveURL(relativeUrl, this.path);
    } else {
      resourcePath = LoaderUtils.extractUrlBase(url);
    }
    this.manager.itemStart(url);
    const _onError = function(e) {
      if (onError) {
        onError(e);
      } else {
        console.error(e);
      }
      scope.manager.itemError(url);
      scope.manager.itemEnd(url);
    };
    const loader = new FileLoader(this.manager);
    loader.setPath(this.path);
    loader.setResponseType("arraybuffer");
    loader.setRequestHeader(this.requestHeader);
    loader.setWithCredentials(this.withCredentials);
    loader.load(url, function(data) {
      try {
        scope.parse(data, resourcePath, function(gltf) {
          onLoad(gltf);
          scope.manager.itemEnd(url);
        }, _onError);
      } catch (e) {
        _onError(e);
      }
    }, onProgress, _onError);
  }
  setDRACOLoader(dracoLoader) {
    this.dracoLoader = dracoLoader;
    return this;
  }
  setDDSLoader() {
    throw new Error(
      'THREE.GLTFLoader: "MSFT_texture_dds" no longer supported. Please update to "KHR_texture_basisu".'
    );
  }
  setKTX2Loader(ktx2Loader) {
    this.ktx2Loader = ktx2Loader;
    return this;
  }
  setMeshoptDecoder(meshoptDecoder) {
    this.meshoptDecoder = meshoptDecoder;
    return this;
  }
  register(callback) {
    if (this.pluginCallbacks.indexOf(callback) === -1) {
      this.pluginCallbacks.push(callback);
    }
    return this;
  }
  unregister(callback) {
    if (this.pluginCallbacks.indexOf(callback) !== -1) {
      this.pluginCallbacks.splice(this.pluginCallbacks.indexOf(callback), 1);
    }
    return this;
  }
  parse(data, path, onLoad, onError) {
    let json;
    const extensions = {};
    const plugins = {};
    const textDecoder = new TextDecoder();
    if (typeof data === "string") {
      json = JSON.parse(data);
    } else if (data instanceof ArrayBuffer) {
      const magic = textDecoder.decode(new Uint8Array(data, 0, 4));
      if (magic === BINARY_EXTENSION_HEADER_MAGIC) {
        try {
          extensions[EXTENSIONS.KHR_BINARY_GLTF] = new GLTFBinaryExtension(data);
        } catch (error) {
          if (onError) onError(error);
          return;
        }
        json = JSON.parse(extensions[EXTENSIONS.KHR_BINARY_GLTF].content);
      } else {
        json = JSON.parse(textDecoder.decode(data));
      }
    } else {
      json = data;
    }
    if (json.asset === void 0 || json.asset.version[0] < 2) {
      if (onError) onError(new Error("THREE.GLTFLoader: Unsupported asset. glTF versions >=2.0 are supported."));
      return;
    }
    const parser = new GLTFParser(json, {
      path: path || this.resourcePath || "",
      crossOrigin: this.crossOrigin,
      requestHeader: this.requestHeader,
      manager: this.manager,
      ktx2Loader: this.ktx2Loader,
      meshoptDecoder: this.meshoptDecoder
    });
    parser.fileLoader.setRequestHeader(this.requestHeader);
    for (let i = 0; i < this.pluginCallbacks.length; i++) {
      const plugin = this.pluginCallbacks[i](parser);
      if (!plugin.name) console.error("THREE.GLTFLoader: Invalid plugin found: missing name");
      plugins[plugin.name] = plugin;
      extensions[plugin.name] = true;
    }
    if (json.extensionsUsed) {
      for (let i = 0; i < json.extensionsUsed.length; ++i) {
        const extensionName = json.extensionsUsed[i];
        const extensionsRequired = json.extensionsRequired || [];
        switch (extensionName) {
          case EXTENSIONS.KHR_MATERIALS_UNLIT:
            extensions[extensionName] = new GLTFMaterialsUnlitExtension();
            break;
          case EXTENSIONS.KHR_DRACO_MESH_COMPRESSION:
            extensions[extensionName] = new GLTFDracoMeshCompressionExtension(json, this.dracoLoader);
            break;
          case EXTENSIONS.KHR_TEXTURE_TRANSFORM:
            extensions[extensionName] = new GLTFTextureTransformExtension();
            break;
          case EXTENSIONS.KHR_MESH_QUANTIZATION:
            extensions[extensionName] = new GLTFMeshQuantizationExtension();
            break;
          default:
            if (extensionsRequired.indexOf(extensionName) >= 0 && plugins[extensionName] === void 0) {
              console.warn('THREE.GLTFLoader: Unknown extension "' + extensionName + '".');
            }
        }
      }
    }
    parser.setExtensions(extensions);
    parser.setPlugins(plugins);
    parser.parse(onLoad, onError);
  }
  parseAsync(data, path) {
    const scope = this;
    return new Promise(function(resolve, reject) {
      scope.parse(data, path, resolve, reject);
    });
  }
};
function GLTFRegistry() {
  let objects = {};
  return {
    get: function(key) {
      return objects[key];
    },
    add: function(key, object) {
      objects[key] = object;
    },
    remove: function(key) {
      delete objects[key];
    },
    removeAll: function() {
      objects = {};
    }
  };
}
var EXTENSIONS = {
  KHR_BINARY_GLTF: "KHR_binary_glTF",
  KHR_DRACO_MESH_COMPRESSION: "KHR_draco_mesh_compression",
  KHR_LIGHTS_PUNCTUAL: "KHR_lights_punctual",
  KHR_MATERIALS_CLEARCOAT: "KHR_materials_clearcoat",
  KHR_MATERIALS_IOR: "KHR_materials_ior",
  KHR_MATERIALS_SHEEN: "KHR_materials_sheen",
  KHR_MATERIALS_SPECULAR: "KHR_materials_specular",
  KHR_MATERIALS_TRANSMISSION: "KHR_materials_transmission",
  KHR_MATERIALS_IRIDESCENCE: "KHR_materials_iridescence",
  KHR_MATERIALS_ANISOTROPY: "KHR_materials_anisotropy",
  KHR_MATERIALS_UNLIT: "KHR_materials_unlit",
  KHR_MATERIALS_VOLUME: "KHR_materials_volume",
  KHR_TEXTURE_BASISU: "KHR_texture_basisu",
  KHR_TEXTURE_TRANSFORM: "KHR_texture_transform",
  KHR_MESH_QUANTIZATION: "KHR_mesh_quantization",
  KHR_MATERIALS_EMISSIVE_STRENGTH: "KHR_materials_emissive_strength",
  EXT_MATERIALS_BUMP: "EXT_materials_bump",
  EXT_TEXTURE_WEBP: "EXT_texture_webp",
  EXT_TEXTURE_AVIF: "EXT_texture_avif",
  EXT_MESHOPT_COMPRESSION: "EXT_meshopt_compression",
  EXT_MESH_GPU_INSTANCING: "EXT_mesh_gpu_instancing"
};
var GLTFLightsExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_LIGHTS_PUNCTUAL;
    this.cache = { refs: {}, uses: {} };
  }
  _markDefs() {
    const parser = this.parser;
    const nodeDefs = this.parser.json.nodes || [];
    for (let nodeIndex = 0, nodeLength = nodeDefs.length; nodeIndex < nodeLength; nodeIndex++) {
      const nodeDef = nodeDefs[nodeIndex];
      if (nodeDef.extensions && nodeDef.extensions[this.name] && nodeDef.extensions[this.name].light !== void 0) {
        parser._addNodeRef(this.cache, nodeDef.extensions[this.name].light);
      }
    }
  }
  _loadLight(lightIndex) {
    const parser = this.parser;
    const cacheKey = "light:" + lightIndex;
    let dependency = parser.cache.get(cacheKey);
    if (dependency) return dependency;
    const json = parser.json;
    const extensions = json.extensions && json.extensions[this.name] || {};
    const lightDefs = extensions.lights || [];
    const lightDef = lightDefs[lightIndex];
    let lightNode;
    const color = new Color(16777215);
    if (lightDef.color !== void 0) color.setRGB(lightDef.color[0], lightDef.color[1], lightDef.color[2], LinearSRGBColorSpace);
    const range = lightDef.range !== void 0 ? lightDef.range : 0;
    switch (lightDef.type) {
      case "directional":
        lightNode = new DirectionalLight(color);
        lightNode.target.position.set(0, 0, -1);
        lightNode.add(lightNode.target);
        break;
      case "point":
        lightNode = new PointLight(color);
        lightNode.distance = range;
        break;
      case "spot":
        lightNode = new SpotLight(color);
        lightNode.distance = range;
        lightDef.spot = lightDef.spot || {};
        lightDef.spot.innerConeAngle = lightDef.spot.innerConeAngle !== void 0 ? lightDef.spot.innerConeAngle : 0;
        lightDef.spot.outerConeAngle = lightDef.spot.outerConeAngle !== void 0 ? lightDef.spot.outerConeAngle : Math.PI / 4;
        lightNode.angle = lightDef.spot.outerConeAngle;
        lightNode.penumbra = 1 - lightDef.spot.innerConeAngle / lightDef.spot.outerConeAngle;
        lightNode.target.position.set(0, 0, -1);
        lightNode.add(lightNode.target);
        break;
      default:
        throw new Error("THREE.GLTFLoader: Unexpected light type: " + lightDef.type);
    }
    lightNode.position.set(0, 0, 0);
    lightNode.decay = 2;
    assignExtrasToUserData(lightNode, lightDef);
    if (lightDef.intensity !== void 0) lightNode.intensity = lightDef.intensity;
    lightNode.name = parser.createUniqueName(lightDef.name || "light_" + lightIndex);
    dependency = Promise.resolve(lightNode);
    parser.cache.add(cacheKey, dependency);
    return dependency;
  }
  getDependency(type, index) {
    if (type !== "light") return;
    return this._loadLight(index);
  }
  createNodeAttachment(nodeIndex) {
    const self2 = this;
    const parser = this.parser;
    const json = parser.json;
    const nodeDef = json.nodes[nodeIndex];
    const lightDef = nodeDef.extensions && nodeDef.extensions[this.name] || {};
    const lightIndex = lightDef.light;
    if (lightIndex === void 0) return null;
    return this._loadLight(lightIndex).then(function(light) {
      return parser._getNodeRef(self2.cache, lightIndex, light);
    });
  }
};
var GLTFMaterialsUnlitExtension = class {
  constructor() {
    this.name = EXTENSIONS.KHR_MATERIALS_UNLIT;
  }
  getMaterialType() {
    return MeshBasicMaterial;
  }
  extendParams(materialParams, materialDef, parser) {
    const pending = [];
    materialParams.color = new Color(1, 1, 1);
    materialParams.opacity = 1;
    const metallicRoughness = materialDef.pbrMetallicRoughness;
    if (metallicRoughness) {
      if (Array.isArray(metallicRoughness.baseColorFactor)) {
        const array = metallicRoughness.baseColorFactor;
        materialParams.color.setRGB(array[0], array[1], array[2], LinearSRGBColorSpace);
        materialParams.opacity = array[3];
      }
      if (metallicRoughness.baseColorTexture !== void 0) {
        pending.push(parser.assignTexture(materialParams, "map", metallicRoughness.baseColorTexture, SRGBColorSpace));
      }
    }
    return Promise.all(pending);
  }
};
var GLTFMaterialsEmissiveStrengthExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_EMISSIVE_STRENGTH;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const emissiveStrength = materialDef.extensions[this.name].emissiveStrength;
    if (emissiveStrength !== void 0) {
      materialParams.emissiveIntensity = emissiveStrength;
    }
    return Promise.resolve();
  }
};
var GLTFMaterialsClearcoatExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_CLEARCOAT;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    const extension = materialDef.extensions[this.name];
    if (extension.clearcoatFactor !== void 0) {
      materialParams.clearcoat = extension.clearcoatFactor;
    }
    if (extension.clearcoatTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "clearcoatMap", extension.clearcoatTexture));
    }
    if (extension.clearcoatRoughnessFactor !== void 0) {
      materialParams.clearcoatRoughness = extension.clearcoatRoughnessFactor;
    }
    if (extension.clearcoatRoughnessTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "clearcoatRoughnessMap", extension.clearcoatRoughnessTexture));
    }
    if (extension.clearcoatNormalTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "clearcoatNormalMap", extension.clearcoatNormalTexture));
      if (extension.clearcoatNormalTexture.scale !== void 0) {
        const scale = extension.clearcoatNormalTexture.scale;
        materialParams.clearcoatNormalScale = new Vector2(scale, scale);
      }
    }
    return Promise.all(pending);
  }
};
var GLTFMaterialsIridescenceExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_IRIDESCENCE;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    const extension = materialDef.extensions[this.name];
    if (extension.iridescenceFactor !== void 0) {
      materialParams.iridescence = extension.iridescenceFactor;
    }
    if (extension.iridescenceTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "iridescenceMap", extension.iridescenceTexture));
    }
    if (extension.iridescenceIor !== void 0) {
      materialParams.iridescenceIOR = extension.iridescenceIor;
    }
    if (materialParams.iridescenceThicknessRange === void 0) {
      materialParams.iridescenceThicknessRange = [100, 400];
    }
    if (extension.iridescenceThicknessMinimum !== void 0) {
      materialParams.iridescenceThicknessRange[0] = extension.iridescenceThicknessMinimum;
    }
    if (extension.iridescenceThicknessMaximum !== void 0) {
      materialParams.iridescenceThicknessRange[1] = extension.iridescenceThicknessMaximum;
    }
    if (extension.iridescenceThicknessTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "iridescenceThicknessMap", extension.iridescenceThicknessTexture));
    }
    return Promise.all(pending);
  }
};
var GLTFMaterialsSheenExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_SHEEN;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    materialParams.sheenColor = new Color(0, 0, 0);
    materialParams.sheenRoughness = 0;
    materialParams.sheen = 1;
    const extension = materialDef.extensions[this.name];
    if (extension.sheenColorFactor !== void 0) {
      const colorFactor = extension.sheenColorFactor;
      materialParams.sheenColor.setRGB(colorFactor[0], colorFactor[1], colorFactor[2], LinearSRGBColorSpace);
    }
    if (extension.sheenRoughnessFactor !== void 0) {
      materialParams.sheenRoughness = extension.sheenRoughnessFactor;
    }
    if (extension.sheenColorTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "sheenColorMap", extension.sheenColorTexture, SRGBColorSpace));
    }
    if (extension.sheenRoughnessTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "sheenRoughnessMap", extension.sheenRoughnessTexture));
    }
    return Promise.all(pending);
  }
};
var GLTFMaterialsTransmissionExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_TRANSMISSION;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    const extension = materialDef.extensions[this.name];
    if (extension.transmissionFactor !== void 0) {
      materialParams.transmission = extension.transmissionFactor;
    }
    if (extension.transmissionTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "transmissionMap", extension.transmissionTexture));
    }
    return Promise.all(pending);
  }
};
var GLTFMaterialsVolumeExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_VOLUME;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    const extension = materialDef.extensions[this.name];
    materialParams.thickness = extension.thicknessFactor !== void 0 ? extension.thicknessFactor : 0;
    if (extension.thicknessTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "thicknessMap", extension.thicknessTexture));
    }
    materialParams.attenuationDistance = extension.attenuationDistance || Infinity;
    const colorArray = extension.attenuationColor || [1, 1, 1];
    materialParams.attenuationColor = new Color().setRGB(colorArray[0], colorArray[1], colorArray[2], LinearSRGBColorSpace);
    return Promise.all(pending);
  }
};
var GLTFMaterialsIorExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_IOR;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const extension = materialDef.extensions[this.name];
    materialParams.ior = extension.ior !== void 0 ? extension.ior : 1.5;
    return Promise.resolve();
  }
};
var GLTFMaterialsSpecularExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_SPECULAR;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    const extension = materialDef.extensions[this.name];
    materialParams.specularIntensity = extension.specularFactor !== void 0 ? extension.specularFactor : 1;
    if (extension.specularTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "specularIntensityMap", extension.specularTexture));
    }
    const colorArray = extension.specularColorFactor || [1, 1, 1];
    materialParams.specularColor = new Color().setRGB(colorArray[0], colorArray[1], colorArray[2], LinearSRGBColorSpace);
    if (extension.specularColorTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "specularColorMap", extension.specularColorTexture, SRGBColorSpace));
    }
    return Promise.all(pending);
  }
};
var GLTFMaterialsBumpExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.EXT_MATERIALS_BUMP;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    const extension = materialDef.extensions[this.name];
    materialParams.bumpScale = extension.bumpFactor !== void 0 ? extension.bumpFactor : 1;
    if (extension.bumpTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "bumpMap", extension.bumpTexture));
    }
    return Promise.all(pending);
  }
};
var GLTFMaterialsAnisotropyExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_MATERIALS_ANISOTROPY;
  }
  getMaterialType(materialIndex) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) return null;
    return MeshPhysicalMaterial;
  }
  extendMaterialParams(materialIndex, materialParams) {
    const parser = this.parser;
    const materialDef = parser.json.materials[materialIndex];
    if (!materialDef.extensions || !materialDef.extensions[this.name]) {
      return Promise.resolve();
    }
    const pending = [];
    const extension = materialDef.extensions[this.name];
    if (extension.anisotropyStrength !== void 0) {
      materialParams.anisotropy = extension.anisotropyStrength;
    }
    if (extension.anisotropyRotation !== void 0) {
      materialParams.anisotropyRotation = extension.anisotropyRotation;
    }
    if (extension.anisotropyTexture !== void 0) {
      pending.push(parser.assignTexture(materialParams, "anisotropyMap", extension.anisotropyTexture));
    }
    return Promise.all(pending);
  }
};
var GLTFTextureBasisUExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.KHR_TEXTURE_BASISU;
  }
  loadTexture(textureIndex) {
    const parser = this.parser;
    const json = parser.json;
    const textureDef = json.textures[textureIndex];
    if (!textureDef.extensions || !textureDef.extensions[this.name]) {
      return null;
    }
    const extension = textureDef.extensions[this.name];
    const loader = parser.options.ktx2Loader;
    if (!loader) {
      if (json.extensionsRequired && json.extensionsRequired.indexOf(this.name) >= 0) {
        throw new Error("THREE.GLTFLoader: setKTX2Loader must be called before loading KTX2 textures");
      } else {
        return null;
      }
    }
    return parser.loadTextureImage(textureIndex, extension.source, loader);
  }
};
var GLTFTextureWebPExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.EXT_TEXTURE_WEBP;
    this.isSupported = null;
  }
  loadTexture(textureIndex) {
    const name2 = this.name;
    const parser = this.parser;
    const json = parser.json;
    const textureDef = json.textures[textureIndex];
    if (!textureDef.extensions || !textureDef.extensions[name2]) {
      return null;
    }
    const extension = textureDef.extensions[name2];
    const source = json.images[extension.source];
    let loader = parser.textureLoader;
    if (source.uri) {
      const handler = parser.options.manager.getHandler(source.uri);
      if (handler !== null) loader = handler;
    }
    return this.detectSupport().then(function(isSupported) {
      if (isSupported) return parser.loadTextureImage(textureIndex, extension.source, loader);
      if (json.extensionsRequired && json.extensionsRequired.indexOf(name2) >= 0) {
        throw new Error("THREE.GLTFLoader: WebP required by asset but unsupported.");
      }
      return parser.loadTexture(textureIndex);
    });
  }
  detectSupport() {
    if (!this.isSupported) {
      this.isSupported = new Promise(function(resolve) {
        const image = new Image();
        image.src = "data:image/webp;base64,UklGRiIAAABXRUJQVlA4IBYAAAAwAQCdASoBAAEADsD+JaQAA3AAAAAA";
        image.onload = image.onerror = function() {
          resolve(image.height === 1);
        };
      });
    }
    return this.isSupported;
  }
};
var GLTFTextureAVIFExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXTENSIONS.EXT_TEXTURE_AVIF;
    this.isSupported = null;
  }
  loadTexture(textureIndex) {
    const name2 = this.name;
    const parser = this.parser;
    const json = parser.json;
    const textureDef = json.textures[textureIndex];
    if (!textureDef.extensions || !textureDef.extensions[name2]) {
      return null;
    }
    const extension = textureDef.extensions[name2];
    const source = json.images[extension.source];
    let loader = parser.textureLoader;
    if (source.uri) {
      const handler = parser.options.manager.getHandler(source.uri);
      if (handler !== null) loader = handler;
    }
    return this.detectSupport().then(function(isSupported) {
      if (isSupported) return parser.loadTextureImage(textureIndex, extension.source, loader);
      if (json.extensionsRequired && json.extensionsRequired.indexOf(name2) >= 0) {
        throw new Error("THREE.GLTFLoader: AVIF required by asset but unsupported.");
      }
      return parser.loadTexture(textureIndex);
    });
  }
  detectSupport() {
    if (!this.isSupported) {
      this.isSupported = new Promise(function(resolve) {
        const image = new Image();
        image.src = "data:image/avif;base64,AAAAIGZ0eXBhdmlmAAAAAGF2aWZtaWYxbWlhZk1BMUIAAADybWV0YQAAAAAAAAAoaGRscgAAAAAAAAAAcGljdAAAAAAAAAAAAAAAAGxpYmF2aWYAAAAADnBpdG0AAAAAAAEAAAAeaWxvYwAAAABEAAABAAEAAAABAAABGgAAABcAAAAoaWluZgAAAAAAAQAAABppbmZlAgAAAAABAABhdjAxQ29sb3IAAAAAamlwcnAAAABLaXBjbwAAABRpc3BlAAAAAAAAAAEAAAABAAAAEHBpeGkAAAAAAwgICAAAAAxhdjFDgQAMAAAAABNjb2xybmNseAACAAIABoAAAAAXaXBtYQAAAAAAAAABAAEEAQKDBAAAAB9tZGF0EgAKCBgABogQEDQgMgkQAAAAB8dSLfI=";
        image.onload = image.onerror = function() {
          resolve(image.height === 1);
        };
      });
    }
    return this.isSupported;
  }
};
var GLTFMeshoptCompression = class {
  constructor(parser) {
    this.name = EXTENSIONS.EXT_MESHOPT_COMPRESSION;
    this.parser = parser;
  }
  loadBufferView(index) {
    const json = this.parser.json;
    const bufferView = json.bufferViews[index];
    if (bufferView.extensions && bufferView.extensions[this.name]) {
      const extensionDef = bufferView.extensions[this.name];
      const buffer = this.parser.getDependency("buffer", extensionDef.buffer);
      const decoder = this.parser.options.meshoptDecoder;
      if (!decoder || !decoder.supported) {
        if (json.extensionsRequired && json.extensionsRequired.indexOf(this.name) >= 0) {
          throw new Error("THREE.GLTFLoader: setMeshoptDecoder must be called before loading compressed files");
        } else {
          return null;
        }
      }
      return buffer.then(function(res) {
        const byteOffset = extensionDef.byteOffset || 0;
        const byteLength = extensionDef.byteLength || 0;
        const count = extensionDef.count;
        const stride = extensionDef.byteStride;
        const source = new Uint8Array(res, byteOffset, byteLength);
        if (decoder.decodeGltfBufferAsync) {
          return decoder.decodeGltfBufferAsync(count, stride, source, extensionDef.mode, extensionDef.filter).then(function(res2) {
            return res2.buffer;
          });
        } else {
          return decoder.ready.then(function() {
            const result = new ArrayBuffer(count * stride);
            decoder.decodeGltfBuffer(new Uint8Array(result), count, stride, source, extensionDef.mode, extensionDef.filter);
            return result;
          });
        }
      });
    } else {
      return null;
    }
  }
};
var GLTFMeshGpuInstancing = class {
  constructor(parser) {
    this.name = EXTENSIONS.EXT_MESH_GPU_INSTANCING;
    this.parser = parser;
  }
  createNodeMesh(nodeIndex) {
    const json = this.parser.json;
    const nodeDef = json.nodes[nodeIndex];
    if (!nodeDef.extensions || !nodeDef.extensions[this.name] || nodeDef.mesh === void 0) {
      return null;
    }
    const meshDef = json.meshes[nodeDef.mesh];
    for (const primitive of meshDef.primitives) {
      if (primitive.mode !== WEBGL_CONSTANTS.TRIANGLES && primitive.mode !== WEBGL_CONSTANTS.TRIANGLE_STRIP && primitive.mode !== WEBGL_CONSTANTS.TRIANGLE_FAN && primitive.mode !== void 0) {
        return null;
      }
    }
    const extensionDef = nodeDef.extensions[this.name];
    const attributesDef = extensionDef.attributes;
    const pending = [];
    const attributes = {};
    for (const key in attributesDef) {
      pending.push(this.parser.getDependency("accessor", attributesDef[key]).then((accessor) => {
        attributes[key] = accessor;
        return attributes[key];
      }));
    }
    if (pending.length < 1) {
      return null;
    }
    pending.push(this.parser.createNodeMesh(nodeIndex));
    return Promise.all(pending).then((results) => {
      const nodeObject = results.pop();
      const meshes = nodeObject.isGroup ? nodeObject.children : [nodeObject];
      const count = results[0].count;
      const instancedMeshes = [];
      for (const mesh of meshes) {
        const m = new Matrix4();
        const p = new Vector32();
        const q = new Quaternion();
        const s = new Vector32(1, 1, 1);
        const instancedMesh = new InstancedMesh(mesh.geometry, mesh.material, count);
        for (let i = 0; i < count; i++) {
          if (attributes.TRANSLATION) {
            p.fromBufferAttribute(attributes.TRANSLATION, i);
          }
          if (attributes.ROTATION) {
            q.fromBufferAttribute(attributes.ROTATION, i);
          }
          if (attributes.SCALE) {
            s.fromBufferAttribute(attributes.SCALE, i);
          }
          instancedMesh.setMatrixAt(i, m.compose(p, q, s));
        }
        for (const attributeName in attributes) {
          if (attributeName === "_COLOR_0") {
            const attr = attributes[attributeName];
            instancedMesh.instanceColor = new InstancedBufferAttribute2(attr.array, attr.itemSize, attr.normalized);
          } else if (attributeName !== "TRANSLATION" && attributeName !== "ROTATION" && attributeName !== "SCALE") {
            mesh.geometry.setAttribute(attributeName, attributes[attributeName]);
          }
        }
        Object3D.prototype.copy.call(instancedMesh, mesh);
        this.parser.assignFinalMaterial(instancedMesh);
        instancedMeshes.push(instancedMesh);
      }
      if (nodeObject.isGroup) {
        nodeObject.clear();
        nodeObject.add(...instancedMeshes);
        return nodeObject;
      }
      return instancedMeshes[0];
    });
  }
};
var BINARY_EXTENSION_HEADER_MAGIC = "glTF";
var BINARY_EXTENSION_HEADER_LENGTH = 12;
var BINARY_EXTENSION_CHUNK_TYPES = { JSON: 1313821514, BIN: 5130562 };
var GLTFBinaryExtension = class {
  constructor(data) {
    this.name = EXTENSIONS.KHR_BINARY_GLTF;
    this.content = null;
    this.body = null;
    const headerView = new DataView(data, 0, BINARY_EXTENSION_HEADER_LENGTH);
    const textDecoder = new TextDecoder();
    this.header = {
      magic: textDecoder.decode(new Uint8Array(data.slice(0, 4))),
      version: headerView.getUint32(4, true),
      length: headerView.getUint32(8, true)
    };
    if (this.header.magic !== BINARY_EXTENSION_HEADER_MAGIC) {
      throw new Error("THREE.GLTFLoader: Unsupported glTF-Binary header.");
    } else if (this.header.version < 2) {
      throw new Error("THREE.GLTFLoader: Legacy binary file detected.");
    }
    const chunkContentsLength = this.header.length - BINARY_EXTENSION_HEADER_LENGTH;
    const chunkView = new DataView(data, BINARY_EXTENSION_HEADER_LENGTH);
    let chunkIndex = 0;
    while (chunkIndex < chunkContentsLength) {
      const chunkLength = chunkView.getUint32(chunkIndex, true);
      chunkIndex += 4;
      const chunkType = chunkView.getUint32(chunkIndex, true);
      chunkIndex += 4;
      if (chunkType === BINARY_EXTENSION_CHUNK_TYPES.JSON) {
        const contentArray = new Uint8Array(data, BINARY_EXTENSION_HEADER_LENGTH + chunkIndex, chunkLength);
        this.content = textDecoder.decode(contentArray);
      } else if (chunkType === BINARY_EXTENSION_CHUNK_TYPES.BIN) {
        const byteOffset = BINARY_EXTENSION_HEADER_LENGTH + chunkIndex;
        this.body = data.slice(byteOffset, byteOffset + chunkLength);
      }
      chunkIndex += chunkLength;
    }
    if (this.content === null) {
      throw new Error("THREE.GLTFLoader: JSON content not found.");
    }
  }
};
var GLTFDracoMeshCompressionExtension = class {
  constructor(json, dracoLoader) {
    if (!dracoLoader) {
      throw new Error("THREE.GLTFLoader: No DRACOLoader instance provided.");
    }
    this.name = EXTENSIONS.KHR_DRACO_MESH_COMPRESSION;
    this.json = json;
    this.dracoLoader = dracoLoader;
    this.dracoLoader.preload();
  }
  decodePrimitive(primitive, parser) {
    const json = this.json;
    const dracoLoader = this.dracoLoader;
    const bufferViewIndex = primitive.extensions[this.name].bufferView;
    const gltfAttributeMap = primitive.extensions[this.name].attributes;
    const threeAttributeMap = {};
    const attributeNormalizedMap = {};
    const attributeTypeMap = {};
    for (const attributeName in gltfAttributeMap) {
      const threeAttributeName = ATTRIBUTES[attributeName] || attributeName.toLowerCase();
      threeAttributeMap[threeAttributeName] = gltfAttributeMap[attributeName];
    }
    for (const attributeName in primitive.attributes) {
      const threeAttributeName = ATTRIBUTES[attributeName] || attributeName.toLowerCase();
      if (gltfAttributeMap[attributeName] !== void 0) {
        const accessorDef = json.accessors[primitive.attributes[attributeName]];
        const componentType = WEBGL_COMPONENT_TYPES[accessorDef.componentType];
        attributeTypeMap[threeAttributeName] = componentType.name;
        attributeNormalizedMap[threeAttributeName] = accessorDef.normalized === true;
      }
    }
    return parser.getDependency("bufferView", bufferViewIndex).then(function(bufferView) {
      return new Promise(function(resolve, reject) {
        dracoLoader.decodeDracoFile(bufferView, function(geometry) {
          for (const attributeName in geometry.attributes) {
            const attribute = geometry.attributes[attributeName];
            const normalized = attributeNormalizedMap[attributeName];
            if (normalized !== void 0) attribute.normalized = normalized;
          }
          resolve(geometry);
        }, threeAttributeMap, attributeTypeMap, LinearSRGBColorSpace, reject);
      });
    });
  }
};
var GLTFTextureTransformExtension = class {
  constructor() {
    this.name = EXTENSIONS.KHR_TEXTURE_TRANSFORM;
  }
  extendTexture(texture, transform) {
    if ((transform.texCoord === void 0 || transform.texCoord === texture.channel) && transform.offset === void 0 && transform.rotation === void 0 && transform.scale === void 0) {
      return texture;
    }
    texture = texture.clone();
    if (transform.texCoord !== void 0) {
      texture.channel = transform.texCoord;
    }
    if (transform.offset !== void 0) {
      texture.offset.fromArray(transform.offset);
    }
    if (transform.rotation !== void 0) {
      texture.rotation = transform.rotation;
    }
    if (transform.scale !== void 0) {
      texture.repeat.fromArray(transform.scale);
    }
    texture.needsUpdate = true;
    return texture;
  }
};
var GLTFMeshQuantizationExtension = class {
  constructor() {
    this.name = EXTENSIONS.KHR_MESH_QUANTIZATION;
  }
};
var GLTFCubicSplineInterpolant = class extends Interpolant {
  constructor(parameterPositions, sampleValues, sampleSize, resultBuffer) {
    super(parameterPositions, sampleValues, sampleSize, resultBuffer);
  }
  copySampleValue_(index) {
    const result = this.resultBuffer, values = this.sampleValues, valueSize = this.valueSize, offset = index * valueSize * 3 + valueSize;
    for (let i = 0; i !== valueSize; i++) {
      result[i] = values[offset + i];
    }
    return result;
  }
  interpolate_(i1, t0, t, t1) {
    const result = this.resultBuffer;
    const values = this.sampleValues;
    const stride = this.valueSize;
    const stride2 = stride * 2;
    const stride3 = stride * 3;
    const td = t1 - t0;
    const p = (t - t0) / td;
    const pp = p * p;
    const ppp = pp * p;
    const offset1 = i1 * stride3;
    const offset0 = offset1 - stride3;
    const s2 = -2 * ppp + 3 * pp;
    const s3 = ppp - pp;
    const s0 = 1 - s2;
    const s1 = s3 - pp + p;
    for (let i = 0; i !== stride; i++) {
      const p0 = values[offset0 + i + stride];
      const m0 = values[offset0 + i + stride2] * td;
      const p1 = values[offset1 + i + stride];
      const m1 = values[offset1 + i] * td;
      result[i] = s0 * p0 + s1 * m0 + s2 * p1 + s3 * m1;
    }
    return result;
  }
};
var _q = new Quaternion();
var GLTFCubicSplineQuaternionInterpolant = class extends GLTFCubicSplineInterpolant {
  interpolate_(i1, t0, t, t1) {
    const result = super.interpolate_(i1, t0, t, t1);
    _q.fromArray(result).normalize().toArray(result);
    return result;
  }
};
var WEBGL_CONSTANTS = {
  FLOAT: 5126,
  //FLOAT_MAT2: 35674,
  FLOAT_MAT3: 35675,
  FLOAT_MAT4: 35676,
  FLOAT_VEC2: 35664,
  FLOAT_VEC3: 35665,
  FLOAT_VEC4: 35666,
  LINEAR: 9729,
  REPEAT: 10497,
  SAMPLER_2D: 35678,
  POINTS: 0,
  LINES: 1,
  LINE_LOOP: 2,
  LINE_STRIP: 3,
  TRIANGLES: 4,
  TRIANGLE_STRIP: 5,
  TRIANGLE_FAN: 6,
  UNSIGNED_BYTE: 5121,
  UNSIGNED_SHORT: 5123
};
var WEBGL_COMPONENT_TYPES = {
  5120: Int8Array,
  5121: Uint8Array,
  5122: Int16Array,
  5123: Uint16Array,
  5125: Uint32Array,
  5126: Float32Array
};
var WEBGL_FILTERS = {
  9728: NearestFilter,
  9729: LinearFilter,
  9984: NearestMipmapNearestFilter,
  9985: LinearMipmapNearestFilter,
  9986: NearestMipmapLinearFilter,
  9987: LinearMipmapLinearFilter
};
var WEBGL_WRAPPINGS = {
  33071: ClampToEdgeWrapping,
  33648: MirroredRepeatWrapping,
  10497: RepeatWrapping
};
var WEBGL_TYPE_SIZES = {
  "SCALAR": 1,
  "VEC2": 2,
  "VEC3": 3,
  "VEC4": 4,
  "MAT2": 4,
  "MAT3": 9,
  "MAT4": 16
};
var ATTRIBUTES = {
  POSITION: "position",
  NORMAL: "normal",
  TANGENT: "tangent",
  TEXCOORD_0: "uv",
  TEXCOORD_1: "uv1",
  TEXCOORD_2: "uv2",
  TEXCOORD_3: "uv3",
  COLOR_0: "color",
  WEIGHTS_0: "skinWeight",
  JOINTS_0: "skinIndex"
};
var PATH_PROPERTIES = {
  scale: "scale",
  translation: "position",
  rotation: "quaternion",
  weights: "morphTargetInfluences"
};
var INTERPOLATION = {
  CUBICSPLINE: void 0,
  // We use a custom interpolant (GLTFCubicSplineInterpolation) for CUBICSPLINE tracks. Each
  // keyframe track will be initialized with a default interpolation type, then modified.
  LINEAR: InterpolateLinear,
  STEP: InterpolateDiscrete
};
var ALPHA_MODES = {
  OPAQUE: "OPAQUE",
  MASK: "MASK",
  BLEND: "BLEND"
};
function createDefaultMaterial(cache) {
  if (cache["DefaultMaterial"] === void 0) {
    cache["DefaultMaterial"] = new MeshStandardMaterial({
      color: 16777215,
      emissive: 0,
      metalness: 1,
      roughness: 1,
      transparent: false,
      depthTest: true,
      side: FrontSide
    });
  }
  return cache["DefaultMaterial"];
}
function addUnknownExtensionsToUserData(knownExtensions, object, objectDef) {
  for (const name2 in objectDef.extensions) {
    if (knownExtensions[name2] === void 0) {
      object.userData.gltfExtensions = object.userData.gltfExtensions || {};
      object.userData.gltfExtensions[name2] = objectDef.extensions[name2];
    }
  }
}
function assignExtrasToUserData(object, gltfDef) {
  if (gltfDef.extras !== void 0) {
    if (typeof gltfDef.extras === "object") {
      Object.assign(object.userData, gltfDef.extras);
    } else {
      console.warn("THREE.GLTFLoader: Ignoring primitive type .extras, " + gltfDef.extras);
    }
  }
}
function addMorphTargets(geometry, targets, parser) {
  let hasMorphPosition = false;
  let hasMorphNormal = false;
  let hasMorphColor = false;
  for (let i = 0, il = targets.length; i < il; i++) {
    const target = targets[i];
    if (target.POSITION !== void 0) hasMorphPosition = true;
    if (target.NORMAL !== void 0) hasMorphNormal = true;
    if (target.COLOR_0 !== void 0) hasMorphColor = true;
    if (hasMorphPosition && hasMorphNormal && hasMorphColor) break;
  }
  if (!hasMorphPosition && !hasMorphNormal && !hasMorphColor) return Promise.resolve(geometry);
  const pendingPositionAccessors = [];
  const pendingNormalAccessors = [];
  const pendingColorAccessors = [];
  for (let i = 0, il = targets.length; i < il; i++) {
    const target = targets[i];
    if (hasMorphPosition) {
      const pendingAccessor = target.POSITION !== void 0 ? parser.getDependency("accessor", target.POSITION) : geometry.attributes.position;
      pendingPositionAccessors.push(pendingAccessor);
    }
    if (hasMorphNormal) {
      const pendingAccessor = target.NORMAL !== void 0 ? parser.getDependency("accessor", target.NORMAL) : geometry.attributes.normal;
      pendingNormalAccessors.push(pendingAccessor);
    }
    if (hasMorphColor) {
      const pendingAccessor = target.COLOR_0 !== void 0 ? parser.getDependency("accessor", target.COLOR_0) : geometry.attributes.color;
      pendingColorAccessors.push(pendingAccessor);
    }
  }
  return Promise.all([
    Promise.all(pendingPositionAccessors),
    Promise.all(pendingNormalAccessors),
    Promise.all(pendingColorAccessors)
  ]).then(function(accessors) {
    const morphPositions = accessors[0];
    const morphNormals = accessors[1];
    const morphColors = accessors[2];
    if (hasMorphPosition) geometry.morphAttributes.position = morphPositions;
    if (hasMorphNormal) geometry.morphAttributes.normal = morphNormals;
    if (hasMorphColor) geometry.morphAttributes.color = morphColors;
    geometry.morphTargetsRelative = true;
    return geometry;
  });
}
function updateMorphTargets(mesh, meshDef) {
  mesh.updateMorphTargets();
  if (meshDef.weights !== void 0) {
    for (let i = 0, il = meshDef.weights.length; i < il; i++) {
      mesh.morphTargetInfluences[i] = meshDef.weights[i];
    }
  }
  if (meshDef.extras && Array.isArray(meshDef.extras.targetNames)) {
    const targetNames = meshDef.extras.targetNames;
    if (mesh.morphTargetInfluences.length === targetNames.length) {
      mesh.morphTargetDictionary = {};
      for (let i = 0, il = targetNames.length; i < il; i++) {
        mesh.morphTargetDictionary[targetNames[i]] = i;
      }
    } else {
      console.warn("THREE.GLTFLoader: Invalid extras.targetNames length. Ignoring names.");
    }
  }
}
function createPrimitiveKey(primitiveDef) {
  let geometryKey;
  const dracoExtension = primitiveDef.extensions && primitiveDef.extensions[EXTENSIONS.KHR_DRACO_MESH_COMPRESSION];
  if (dracoExtension) {
    geometryKey = "draco:" + dracoExtension.bufferView + ":" + dracoExtension.indices + ":" + createAttributesKey(dracoExtension.attributes);
  } else {
    geometryKey = primitiveDef.indices + ":" + createAttributesKey(primitiveDef.attributes) + ":" + primitiveDef.mode;
  }
  if (primitiveDef.targets !== void 0) {
    for (let i = 0, il = primitiveDef.targets.length; i < il; i++) {
      geometryKey += ":" + createAttributesKey(primitiveDef.targets[i]);
    }
  }
  return geometryKey;
}
function createAttributesKey(attributes) {
  let attributesKey = "";
  const keys = Object.keys(attributes).sort();
  for (let i = 0, il = keys.length; i < il; i++) {
    attributesKey += keys[i] + ":" + attributes[keys[i]] + ";";
  }
  return attributesKey;
}
function getNormalizedComponentScale(constructor) {
  switch (constructor) {
    case Int8Array:
      return 1 / 127;
    case Uint8Array:
      return 1 / 255;
    case Int16Array:
      return 1 / 32767;
    case Uint16Array:
      return 1 / 65535;
    default:
      throw new Error("THREE.GLTFLoader: Unsupported normalized accessor component type.");
  }
}
function getImageURIMimeType(uri) {
  if (uri.search(/\.jpe?g($|\?)/i) > 0 || uri.search(/^data\:image\/jpeg/) === 0) return "image/jpeg";
  if (uri.search(/\.webp($|\?)/i) > 0 || uri.search(/^data\:image\/webp/) === 0) return "image/webp";
  return "image/png";
}
var _identityMatrix = new Matrix4();
var GLTFParser = class {
  constructor(json = {}, options = {}) {
    this.json = json;
    this.extensions = {};
    this.plugins = {};
    this.options = options;
    this.cache = new GLTFRegistry();
    this.associations = /* @__PURE__ */ new Map();
    this.primitiveCache = {};
    this.nodeCache = {};
    this.meshCache = { refs: {}, uses: {} };
    this.cameraCache = { refs: {}, uses: {} };
    this.lightCache = { refs: {}, uses: {} };
    this.sourceCache = {};
    this.textureCache = {};
    this.nodeNamesUsed = {};
    let isSafari = false;
    let isFirefox = false;
    let firefoxVersion = -1;
    if (typeof navigator !== "undefined") {
      isSafari = /^((?!chrome|android).)*safari/i.test(navigator.userAgent) === true;
      isFirefox = navigator.userAgent.indexOf("Firefox") > -1;
      firefoxVersion = isFirefox ? navigator.userAgent.match(/Firefox\/([0-9]+)\./)[1] : -1;
    }
    if (typeof createImageBitmap === "undefined" || isSafari || isFirefox && firefoxVersion < 98) {
      this.textureLoader = new TextureLoader(this.options.manager);
    } else {
      this.textureLoader = new ImageBitmapLoader(this.options.manager);
    }
    this.textureLoader.setCrossOrigin(this.options.crossOrigin);
    this.textureLoader.setRequestHeader(this.options.requestHeader);
    this.fileLoader = new FileLoader(this.options.manager);
    this.fileLoader.setResponseType("arraybuffer");
    if (this.options.crossOrigin === "use-credentials") {
      this.fileLoader.setWithCredentials(true);
    }
  }
  setExtensions(extensions) {
    this.extensions = extensions;
  }
  setPlugins(plugins) {
    this.plugins = plugins;
  }
  parse(onLoad, onError) {
    const parser = this;
    const json = this.json;
    const extensions = this.extensions;
    this.cache.removeAll();
    this.nodeCache = {};
    this._invokeAll(function(ext) {
      return ext._markDefs && ext._markDefs();
    });
    Promise.all(this._invokeAll(function(ext) {
      return ext.beforeRoot && ext.beforeRoot();
    })).then(function() {
      return Promise.all([
        parser.getDependencies("scene"),
        parser.getDependencies("animation"),
        parser.getDependencies("camera")
      ]);
    }).then(function(dependencies) {
      const result = {
        scene: dependencies[0][json.scene || 0],
        scenes: dependencies[0],
        animations: dependencies[1],
        cameras: dependencies[2],
        asset: json.asset,
        parser,
        userData: {}
      };
      addUnknownExtensionsToUserData(extensions, result, json);
      assignExtrasToUserData(result, json);
      return Promise.all(parser._invokeAll(function(ext) {
        return ext.afterRoot && ext.afterRoot(result);
      })).then(function() {
        onLoad(result);
      });
    }).catch(onError);
  }
  /**
   * Marks the special nodes/meshes in json for efficient parse.
   */
  _markDefs() {
    const nodeDefs = this.json.nodes || [];
    const skinDefs = this.json.skins || [];
    const meshDefs = this.json.meshes || [];
    for (let skinIndex = 0, skinLength = skinDefs.length; skinIndex < skinLength; skinIndex++) {
      const joints = skinDefs[skinIndex].joints;
      for (let i = 0, il = joints.length; i < il; i++) {
        nodeDefs[joints[i]].isBone = true;
      }
    }
    for (let nodeIndex = 0, nodeLength = nodeDefs.length; nodeIndex < nodeLength; nodeIndex++) {
      const nodeDef = nodeDefs[nodeIndex];
      if (nodeDef.mesh !== void 0) {
        this._addNodeRef(this.meshCache, nodeDef.mesh);
        if (nodeDef.skin !== void 0) {
          meshDefs[nodeDef.mesh].isSkinnedMesh = true;
        }
      }
      if (nodeDef.camera !== void 0) {
        this._addNodeRef(this.cameraCache, nodeDef.camera);
      }
    }
  }
  /**
   * Counts references to shared node / Object3D resources. These resources
   * can be reused, or "instantiated", at multiple nodes in the scene
   * hierarchy. Mesh, Camera, and Light instances are instantiated and must
   * be marked. Non-scenegraph resources (like Materials, Geometries, and
   * Textures) can be reused directly and are not marked here.
   *
   * Example: CesiumMilkTruck sample model reuses "Wheel" meshes.
   */
  _addNodeRef(cache, index) {
    if (index === void 0) return;
    if (cache.refs[index] === void 0) {
      cache.refs[index] = cache.uses[index] = 0;
    }
    cache.refs[index]++;
  }
  /** Returns a reference to a shared resource, cloning it if necessary. */
  _getNodeRef(cache, index, object) {
    if (cache.refs[index] <= 1) return object;
    const ref = object.clone();
    const updateMappings = (original, clone) => {
      const mappings = this.associations.get(original);
      if (mappings != null) {
        this.associations.set(clone, mappings);
      }
      for (const [i, child] of original.children.entries()) {
        updateMappings(child, clone.children[i]);
      }
    };
    updateMappings(object, ref);
    ref.name += "_instance_" + cache.uses[index]++;
    return ref;
  }
  _invokeOne(func) {
    const extensions = Object.values(this.plugins);
    extensions.push(this);
    for (let i = 0; i < extensions.length; i++) {
      const result = func(extensions[i]);
      if (result) return result;
    }
    return null;
  }
  _invokeAll(func) {
    const extensions = Object.values(this.plugins);
    extensions.unshift(this);
    const pending = [];
    for (let i = 0; i < extensions.length; i++) {
      const result = func(extensions[i]);
      if (result) pending.push(result);
    }
    return pending;
  }
  /**
   * Requests the specified dependency asynchronously, with caching.
   * @param {string} type
   * @param {number} index
   * @return {Promise<Object3D|Material|THREE.Texture|AnimationClip|ArrayBuffer|Object>}
   */
  getDependency(type, index) {
    const cacheKey = type + ":" + index;
    let dependency = this.cache.get(cacheKey);
    if (!dependency) {
      switch (type) {
        case "scene":
          dependency = this.loadScene(index);
          break;
        case "node":
          dependency = this._invokeOne(function(ext) {
            return ext.loadNode && ext.loadNode(index);
          });
          break;
        case "mesh":
          dependency = this._invokeOne(function(ext) {
            return ext.loadMesh && ext.loadMesh(index);
          });
          break;
        case "accessor":
          dependency = this.loadAccessor(index);
          break;
        case "bufferView":
          dependency = this._invokeOne(function(ext) {
            return ext.loadBufferView && ext.loadBufferView(index);
          });
          break;
        case "buffer":
          dependency = this.loadBuffer(index);
          break;
        case "material":
          dependency = this._invokeOne(function(ext) {
            return ext.loadMaterial && ext.loadMaterial(index);
          });
          break;
        case "texture":
          dependency = this._invokeOne(function(ext) {
            return ext.loadTexture && ext.loadTexture(index);
          });
          break;
        case "skin":
          dependency = this.loadSkin(index);
          break;
        case "animation":
          dependency = this._invokeOne(function(ext) {
            return ext.loadAnimation && ext.loadAnimation(index);
          });
          break;
        case "camera":
          dependency = this.loadCamera(index);
          break;
        default:
          dependency = this._invokeOne(function(ext) {
            return ext != this && ext.getDependency && ext.getDependency(type, index);
          });
          if (!dependency) {
            throw new Error("Unknown type: " + type);
          }
          break;
      }
      this.cache.add(cacheKey, dependency);
    }
    return dependency;
  }
  /**
   * Requests all dependencies of the specified type asynchronously, with caching.
   * @param {string} type
   * @return {Promise<Array<Object>>}
   */
  getDependencies(type) {
    let dependencies = this.cache.get(type);
    if (!dependencies) {
      const parser = this;
      const defs = this.json[type + (type === "mesh" ? "es" : "s")] || [];
      dependencies = Promise.all(defs.map(function(def, index) {
        return parser.getDependency(type, index);
      }));
      this.cache.add(type, dependencies);
    }
    return dependencies;
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#buffers-and-buffer-views
   * @param {number} bufferIndex
   * @return {Promise<ArrayBuffer>}
   */
  loadBuffer(bufferIndex) {
    const bufferDef = this.json.buffers[bufferIndex];
    const loader = this.fileLoader;
    if (bufferDef.type && bufferDef.type !== "arraybuffer") {
      throw new Error("THREE.GLTFLoader: " + bufferDef.type + " buffer type is not supported.");
    }
    if (bufferDef.uri === void 0 && bufferIndex === 0) {
      return Promise.resolve(this.extensions[EXTENSIONS.KHR_BINARY_GLTF].body);
    }
    const options = this.options;
    return new Promise(function(resolve, reject) {
      loader.load(LoaderUtils.resolveURL(bufferDef.uri, options.path), resolve, void 0, function() {
        reject(new Error('THREE.GLTFLoader: Failed to load buffer "' + bufferDef.uri + '".'));
      });
    });
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#buffers-and-buffer-views
   * @param {number} bufferViewIndex
   * @return {Promise<ArrayBuffer>}
   */
  loadBufferView(bufferViewIndex) {
    const bufferViewDef = this.json.bufferViews[bufferViewIndex];
    return this.getDependency("buffer", bufferViewDef.buffer).then(function(buffer) {
      const byteLength = bufferViewDef.byteLength || 0;
      const byteOffset = bufferViewDef.byteOffset || 0;
      return buffer.slice(byteOffset, byteOffset + byteLength);
    });
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#accessors
   * @param {number} accessorIndex
   * @return {Promise<BufferAttribute|InterleavedBufferAttribute>}
   */
  loadAccessor(accessorIndex) {
    const parser = this;
    const json = this.json;
    const accessorDef = this.json.accessors[accessorIndex];
    if (accessorDef.bufferView === void 0 && accessorDef.sparse === void 0) {
      const itemSize = WEBGL_TYPE_SIZES[accessorDef.type];
      const TypedArray = WEBGL_COMPONENT_TYPES[accessorDef.componentType];
      const normalized = accessorDef.normalized === true;
      const array = new TypedArray(accessorDef.count * itemSize);
      return Promise.resolve(new BufferAttribute2(array, itemSize, normalized));
    }
    const pendingBufferViews = [];
    if (accessorDef.bufferView !== void 0) {
      pendingBufferViews.push(this.getDependency("bufferView", accessorDef.bufferView));
    } else {
      pendingBufferViews.push(null);
    }
    if (accessorDef.sparse !== void 0) {
      pendingBufferViews.push(this.getDependency("bufferView", accessorDef.sparse.indices.bufferView));
      pendingBufferViews.push(this.getDependency("bufferView", accessorDef.sparse.values.bufferView));
    }
    return Promise.all(pendingBufferViews).then(function(bufferViews) {
      const bufferView = bufferViews[0];
      const itemSize = WEBGL_TYPE_SIZES[accessorDef.type];
      const TypedArray = WEBGL_COMPONENT_TYPES[accessorDef.componentType];
      const elementBytes = TypedArray.BYTES_PER_ELEMENT;
      const itemBytes = elementBytes * itemSize;
      const byteOffset = accessorDef.byteOffset || 0;
      const byteStride = accessorDef.bufferView !== void 0 ? json.bufferViews[accessorDef.bufferView].byteStride : void 0;
      const normalized = accessorDef.normalized === true;
      let array, bufferAttribute;
      if (byteStride && byteStride !== itemBytes) {
        const ibSlice = Math.floor(byteOffset / byteStride);
        const ibCacheKey = "InterleavedBuffer:" + accessorDef.bufferView + ":" + accessorDef.componentType + ":" + ibSlice + ":" + accessorDef.count;
        let ib = parser.cache.get(ibCacheKey);
        if (!ib) {
          array = new TypedArray(bufferView, ibSlice * byteStride, accessorDef.count * byteStride / elementBytes);
          ib = new InterleavedBuffer2(array, byteStride / elementBytes);
          parser.cache.add(ibCacheKey, ib);
        }
        bufferAttribute = new InterleavedBufferAttribute2(ib, itemSize, byteOffset % byteStride / elementBytes, normalized);
      } else {
        if (bufferView === null) {
          array = new TypedArray(accessorDef.count * itemSize);
        } else {
          array = new TypedArray(bufferView, byteOffset, accessorDef.count * itemSize);
        }
        bufferAttribute = new BufferAttribute2(array, itemSize, normalized);
      }
      if (accessorDef.sparse !== void 0) {
        const itemSizeIndices = WEBGL_TYPE_SIZES.SCALAR;
        const TypedArrayIndices = WEBGL_COMPONENT_TYPES[accessorDef.sparse.indices.componentType];
        const byteOffsetIndices = accessorDef.sparse.indices.byteOffset || 0;
        const byteOffsetValues = accessorDef.sparse.values.byteOffset || 0;
        const sparseIndices = new TypedArrayIndices(bufferViews[1], byteOffsetIndices, accessorDef.sparse.count * itemSizeIndices);
        const sparseValues = new TypedArray(bufferViews[2], byteOffsetValues, accessorDef.sparse.count * itemSize);
        if (bufferView !== null) {
          bufferAttribute = new BufferAttribute2(bufferAttribute.array.slice(), bufferAttribute.itemSize, bufferAttribute.normalized);
        }
        for (let i = 0, il = sparseIndices.length; i < il; i++) {
          const index = sparseIndices[i];
          bufferAttribute.setX(index, sparseValues[i * itemSize]);
          if (itemSize >= 2) bufferAttribute.setY(index, sparseValues[i * itemSize + 1]);
          if (itemSize >= 3) bufferAttribute.setZ(index, sparseValues[i * itemSize + 2]);
          if (itemSize >= 4) bufferAttribute.setW(index, sparseValues[i * itemSize + 3]);
          if (itemSize >= 5) throw new Error("THREE.GLTFLoader: Unsupported itemSize in sparse BufferAttribute.");
        }
      }
      return bufferAttribute;
    });
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#textures
   * @param {number} textureIndex
   * @return {Promise<THREE.Texture|null>}
   */
  loadTexture(textureIndex) {
    const json = this.json;
    const options = this.options;
    const textureDef = json.textures[textureIndex];
    const sourceIndex = textureDef.source;
    const sourceDef = json.images[sourceIndex];
    let loader = this.textureLoader;
    if (sourceDef.uri) {
      const handler = options.manager.getHandler(sourceDef.uri);
      if (handler !== null) loader = handler;
    }
    return this.loadTextureImage(textureIndex, sourceIndex, loader);
  }
  loadTextureImage(textureIndex, sourceIndex, loader) {
    const parser = this;
    const json = this.json;
    const textureDef = json.textures[textureIndex];
    const sourceDef = json.images[sourceIndex];
    const cacheKey = (sourceDef.uri || sourceDef.bufferView) + ":" + textureDef.sampler;
    if (this.textureCache[cacheKey]) {
      return this.textureCache[cacheKey];
    }
    const promise = this.loadImageSource(sourceIndex, loader).then(function(texture) {
      texture.flipY = false;
      texture.name = textureDef.name || sourceDef.name || "";
      if (texture.name === "" && typeof sourceDef.uri === "string" && sourceDef.uri.startsWith("data:image/") === false) {
        texture.name = sourceDef.uri;
      }
      const samplers = json.samplers || {};
      const sampler = samplers[textureDef.sampler] || {};
      texture.magFilter = WEBGL_FILTERS[sampler.magFilter] || LinearFilter;
      texture.minFilter = WEBGL_FILTERS[sampler.minFilter] || LinearMipmapLinearFilter;
      texture.wrapS = WEBGL_WRAPPINGS[sampler.wrapS] || RepeatWrapping;
      texture.wrapT = WEBGL_WRAPPINGS[sampler.wrapT] || RepeatWrapping;
      parser.associations.set(texture, { textures: textureIndex });
      return texture;
    }).catch(function() {
      return null;
    });
    this.textureCache[cacheKey] = promise;
    return promise;
  }
  loadImageSource(sourceIndex, loader) {
    const parser = this;
    const json = this.json;
    const options = this.options;
    if (this.sourceCache[sourceIndex] !== void 0) {
      return this.sourceCache[sourceIndex].then((texture) => texture.clone());
    }
    const sourceDef = json.images[sourceIndex];
    const URL2 = self.URL || self.webkitURL;
    let sourceURI = sourceDef.uri || "";
    let isObjectURL = false;
    if (sourceDef.bufferView !== void 0) {
      sourceURI = parser.getDependency("bufferView", sourceDef.bufferView).then(function(bufferView) {
        isObjectURL = true;
        const blob = new Blob([bufferView], { type: sourceDef.mimeType });
        sourceURI = URL2.createObjectURL(blob);
        return sourceURI;
      });
    } else if (sourceDef.uri === void 0) {
      throw new Error("THREE.GLTFLoader: Image " + sourceIndex + " is missing URI and bufferView");
    }
    const promise = Promise.resolve(sourceURI).then(function(sourceURI2) {
      return new Promise(function(resolve, reject) {
        let onLoad = resolve;
        if (loader.isImageBitmapLoader === true) {
          onLoad = function(imageBitmap) {
            const texture = new Texture(imageBitmap);
            texture.needsUpdate = true;
            resolve(texture);
          };
        }
        loader.load(LoaderUtils.resolveURL(sourceURI2, options.path), onLoad, void 0, reject);
      });
    }).then(function(texture) {
      if (isObjectURL === true) {
        URL2.revokeObjectURL(sourceURI);
      }
      texture.userData.mimeType = sourceDef.mimeType || getImageURIMimeType(sourceDef.uri);
      return texture;
    }).catch(function(error) {
      console.error("THREE.GLTFLoader: Couldn't load texture", sourceURI);
      throw error;
    });
    this.sourceCache[sourceIndex] = promise;
    return promise;
  }
  /**
   * Asynchronously assigns a texture to the given material parameters.
   * @param {Object} materialParams
   * @param {string} mapName
   * @param {Object} mapDef
   * @return {Promise<Texture>}
   */
  assignTexture(materialParams, mapName, mapDef, colorSpace) {
    const parser = this;
    return this.getDependency("texture", mapDef.index).then(function(texture) {
      if (!texture) return null;
      if (mapDef.texCoord !== void 0 && mapDef.texCoord > 0) {
        texture = texture.clone();
        texture.channel = mapDef.texCoord;
      }
      if (parser.extensions[EXTENSIONS.KHR_TEXTURE_TRANSFORM]) {
        const transform = mapDef.extensions !== void 0 ? mapDef.extensions[EXTENSIONS.KHR_TEXTURE_TRANSFORM] : void 0;
        if (transform) {
          const gltfReference = parser.associations.get(texture);
          texture = parser.extensions[EXTENSIONS.KHR_TEXTURE_TRANSFORM].extendTexture(texture, transform);
          parser.associations.set(texture, gltfReference);
        }
      }
      if (colorSpace !== void 0) {
        texture.colorSpace = colorSpace;
      }
      materialParams[mapName] = texture;
      return texture;
    });
  }
  /**
   * Assigns final material to a Mesh, Line, or Points instance. The instance
   * already has a material (generated from the glTF material options alone)
   * but reuse of the same glTF material may require multiple threejs materials
   * to accommodate different primitive types, defines, etc. New materials will
   * be created if necessary, and reused from a cache.
   * @param  {Object3D} mesh Mesh, Line, or Points instance.
   */
  assignFinalMaterial(mesh) {
    const geometry = mesh.geometry;
    let material = mesh.material;
    const useDerivativeTangents = geometry.attributes.tangent === void 0;
    const useVertexColors = geometry.attributes.color !== void 0;
    const useFlatShading = geometry.attributes.normal === void 0;
    if (mesh.isPoints) {
      const cacheKey = "PointsMaterial:" + material.uuid;
      let pointsMaterial = this.cache.get(cacheKey);
      if (!pointsMaterial) {
        pointsMaterial = new PointsMaterial();
        Material.prototype.copy.call(pointsMaterial, material);
        pointsMaterial.color.copy(material.color);
        pointsMaterial.map = material.map;
        pointsMaterial.sizeAttenuation = false;
        this.cache.add(cacheKey, pointsMaterial);
      }
      material = pointsMaterial;
    } else if (mesh.isLine) {
      const cacheKey = "LineBasicMaterial:" + material.uuid;
      let lineMaterial = this.cache.get(cacheKey);
      if (!lineMaterial) {
        lineMaterial = new LineBasicMaterial();
        Material.prototype.copy.call(lineMaterial, material);
        lineMaterial.color.copy(material.color);
        lineMaterial.map = material.map;
        this.cache.add(cacheKey, lineMaterial);
      }
      material = lineMaterial;
    }
    if (useDerivativeTangents || useVertexColors || useFlatShading) {
      let cacheKey = "ClonedMaterial:" + material.uuid + ":";
      if (useDerivativeTangents) cacheKey += "derivative-tangents:";
      if (useVertexColors) cacheKey += "vertex-colors:";
      if (useFlatShading) cacheKey += "flat-shading:";
      let cachedMaterial = this.cache.get(cacheKey);
      if (!cachedMaterial) {
        cachedMaterial = material.clone();
        if (useVertexColors) cachedMaterial.vertexColors = true;
        if (useFlatShading) cachedMaterial.flatShading = true;
        if (useDerivativeTangents) {
          if (cachedMaterial.normalScale) cachedMaterial.normalScale.y *= -1;
          if (cachedMaterial.clearcoatNormalScale) cachedMaterial.clearcoatNormalScale.y *= -1;
        }
        this.cache.add(cacheKey, cachedMaterial);
        this.associations.set(cachedMaterial, this.associations.get(material));
      }
      material = cachedMaterial;
    }
    mesh.material = material;
  }
  getMaterialType() {
    return MeshStandardMaterial;
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#materials
   * @param {number} materialIndex
   * @return {Promise<Material>}
   */
  loadMaterial(materialIndex) {
    const parser = this;
    const json = this.json;
    const extensions = this.extensions;
    const materialDef = json.materials[materialIndex];
    let materialType;
    const materialParams = {};
    const materialExtensions = materialDef.extensions || {};
    const pending = [];
    if (materialExtensions[EXTENSIONS.KHR_MATERIALS_UNLIT]) {
      const kmuExtension = extensions[EXTENSIONS.KHR_MATERIALS_UNLIT];
      materialType = kmuExtension.getMaterialType();
      pending.push(kmuExtension.extendParams(materialParams, materialDef, parser));
    } else {
      const metallicRoughness = materialDef.pbrMetallicRoughness || {};
      materialParams.color = new Color(1, 1, 1);
      materialParams.opacity = 1;
      if (Array.isArray(metallicRoughness.baseColorFactor)) {
        const array = metallicRoughness.baseColorFactor;
        materialParams.color.setRGB(array[0], array[1], array[2], LinearSRGBColorSpace);
        materialParams.opacity = array[3];
      }
      if (metallicRoughness.baseColorTexture !== void 0) {
        pending.push(parser.assignTexture(materialParams, "map", metallicRoughness.baseColorTexture, SRGBColorSpace));
      }
      materialParams.metalness = metallicRoughness.metallicFactor !== void 0 ? metallicRoughness.metallicFactor : 1;
      materialParams.roughness = metallicRoughness.roughnessFactor !== void 0 ? metallicRoughness.roughnessFactor : 1;
      if (metallicRoughness.metallicRoughnessTexture !== void 0) {
        pending.push(parser.assignTexture(materialParams, "metalnessMap", metallicRoughness.metallicRoughnessTexture));
        pending.push(parser.assignTexture(materialParams, "roughnessMap", metallicRoughness.metallicRoughnessTexture));
      }
      materialType = this._invokeOne(function(ext) {
        return ext.getMaterialType && ext.getMaterialType(materialIndex);
      });
      pending.push(Promise.all(this._invokeAll(function(ext) {
        return ext.extendMaterialParams && ext.extendMaterialParams(materialIndex, materialParams);
      })));
    }
    if (materialDef.doubleSided === true) {
      materialParams.side = DoubleSide;
    }
    const alphaMode = materialDef.alphaMode || ALPHA_MODES.OPAQUE;
    if (alphaMode === ALPHA_MODES.BLEND) {
      materialParams.transparent = true;
      materialParams.depthWrite = false;
    } else {
      materialParams.transparent = false;
      if (alphaMode === ALPHA_MODES.MASK) {
        materialParams.alphaTest = materialDef.alphaCutoff !== void 0 ? materialDef.alphaCutoff : 0.5;
      }
    }
    if (materialDef.normalTexture !== void 0 && materialType !== MeshBasicMaterial) {
      pending.push(parser.assignTexture(materialParams, "normalMap", materialDef.normalTexture));
      materialParams.normalScale = new Vector2(1, 1);
      if (materialDef.normalTexture.scale !== void 0) {
        const scale = materialDef.normalTexture.scale;
        materialParams.normalScale.set(scale, scale);
      }
    }
    if (materialDef.occlusionTexture !== void 0 && materialType !== MeshBasicMaterial) {
      pending.push(parser.assignTexture(materialParams, "aoMap", materialDef.occlusionTexture));
      if (materialDef.occlusionTexture.strength !== void 0) {
        materialParams.aoMapIntensity = materialDef.occlusionTexture.strength;
      }
    }
    if (materialDef.emissiveFactor !== void 0 && materialType !== MeshBasicMaterial) {
      const emissiveFactor = materialDef.emissiveFactor;
      materialParams.emissive = new Color().setRGB(emissiveFactor[0], emissiveFactor[1], emissiveFactor[2], LinearSRGBColorSpace);
    }
    if (materialDef.emissiveTexture !== void 0 && materialType !== MeshBasicMaterial) {
      pending.push(parser.assignTexture(materialParams, "emissiveMap", materialDef.emissiveTexture, SRGBColorSpace));
    }
    return Promise.all(pending).then(function() {
      const material = new materialType(materialParams);
      if (materialDef.name) material.name = materialDef.name;
      assignExtrasToUserData(material, materialDef);
      parser.associations.set(material, { materials: materialIndex });
      if (materialDef.extensions) addUnknownExtensionsToUserData(extensions, material, materialDef);
      return material;
    });
  }
  /** When Object3D instances are targeted by animation, they need unique names. */
  createUniqueName(originalName) {
    const sanitizedName = PropertyBinding.sanitizeNodeName(originalName || "");
    if (sanitizedName in this.nodeNamesUsed) {
      return sanitizedName + "_" + ++this.nodeNamesUsed[sanitizedName];
    } else {
      this.nodeNamesUsed[sanitizedName] = 0;
      return sanitizedName;
    }
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#geometry
   *
   * Creates BufferGeometries from primitives.
   *
   * @param {Array<GLTF.Primitive>} primitives
   * @return {Promise<Array<BufferGeometry>>}
   */
  loadGeometries(primitives) {
    const parser = this;
    const extensions = this.extensions;
    const cache = this.primitiveCache;
    function createDracoPrimitive(primitive) {
      return extensions[EXTENSIONS.KHR_DRACO_MESH_COMPRESSION].decodePrimitive(primitive, parser).then(function(geometry) {
        return addPrimitiveAttributes(geometry, primitive, parser);
      });
    }
    const pending = [];
    for (let i = 0, il = primitives.length; i < il; i++) {
      const primitive = primitives[i];
      const cacheKey = createPrimitiveKey(primitive);
      const cached = cache[cacheKey];
      if (cached) {
        pending.push(cached.promise);
      } else {
        let geometryPromise;
        if (primitive.extensions && primitive.extensions[EXTENSIONS.KHR_DRACO_MESH_COMPRESSION]) {
          geometryPromise = createDracoPrimitive(primitive);
        } else {
          geometryPromise = addPrimitiveAttributes(new BufferGeometry2(), primitive, parser);
        }
        cache[cacheKey] = { primitive, promise: geometryPromise };
        pending.push(geometryPromise);
      }
    }
    return Promise.all(pending);
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#meshes
   * @param {number} meshIndex
   * @return {Promise<Group|Mesh|SkinnedMesh>}
   */
  loadMesh(meshIndex) {
    const parser = this;
    const json = this.json;
    const extensions = this.extensions;
    const meshDef = json.meshes[meshIndex];
    const primitives = meshDef.primitives;
    const pending = [];
    for (let i = 0, il = primitives.length; i < il; i++) {
      const material = primitives[i].material === void 0 ? createDefaultMaterial(this.cache) : this.getDependency("material", primitives[i].material);
      pending.push(material);
    }
    pending.push(parser.loadGeometries(primitives));
    return Promise.all(pending).then(function(results) {
      const materials = results.slice(0, results.length - 1);
      const geometries = results[results.length - 1];
      const meshes = [];
      for (let i = 0, il = geometries.length; i < il; i++) {
        const geometry = geometries[i];
        const primitive = primitives[i];
        let mesh;
        const material = materials[i];
        if (primitive.mode === WEBGL_CONSTANTS.TRIANGLES || primitive.mode === WEBGL_CONSTANTS.TRIANGLE_STRIP || primitive.mode === WEBGL_CONSTANTS.TRIANGLE_FAN || primitive.mode === void 0) {
          mesh = meshDef.isSkinnedMesh === true ? new SkinnedMesh(geometry, material) : new Mesh(geometry, material);
          if (mesh.isSkinnedMesh === true) {
            mesh.normalizeSkinWeights();
          }
          if (primitive.mode === WEBGL_CONSTANTS.TRIANGLE_STRIP) {
            mesh.geometry = toTrianglesDrawMode(mesh.geometry, TriangleStripDrawMode2);
          } else if (primitive.mode === WEBGL_CONSTANTS.TRIANGLE_FAN) {
            mesh.geometry = toTrianglesDrawMode(mesh.geometry, TriangleFanDrawMode2);
          }
        } else if (primitive.mode === WEBGL_CONSTANTS.LINES) {
          mesh = new LineSegments(geometry, material);
        } else if (primitive.mode === WEBGL_CONSTANTS.LINE_STRIP) {
          mesh = new Line(geometry, material);
        } else if (primitive.mode === WEBGL_CONSTANTS.LINE_LOOP) {
          mesh = new LineLoop(geometry, material);
        } else if (primitive.mode === WEBGL_CONSTANTS.POINTS) {
          mesh = new Points(geometry, material);
        } else {
          throw new Error("THREE.GLTFLoader: Primitive mode unsupported: " + primitive.mode);
        }
        if (Object.keys(mesh.geometry.morphAttributes).length > 0) {
          updateMorphTargets(mesh, meshDef);
        }
        mesh.name = parser.createUniqueName(meshDef.name || "mesh_" + meshIndex);
        assignExtrasToUserData(mesh, meshDef);
        if (primitive.extensions) addUnknownExtensionsToUserData(extensions, mesh, primitive);
        parser.assignFinalMaterial(mesh);
        meshes.push(mesh);
      }
      for (let i = 0, il = meshes.length; i < il; i++) {
        parser.associations.set(meshes[i], {
          meshes: meshIndex,
          primitives: i
        });
      }
      if (meshes.length === 1) {
        if (meshDef.extensions) addUnknownExtensionsToUserData(extensions, meshes[0], meshDef);
        return meshes[0];
      }
      const group = new Group();
      if (meshDef.extensions) addUnknownExtensionsToUserData(extensions, group, meshDef);
      parser.associations.set(group, { meshes: meshIndex });
      for (let i = 0, il = meshes.length; i < il; i++) {
        group.add(meshes[i]);
      }
      return group;
    });
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#cameras
   * @param {number} cameraIndex
   * @return {Promise<THREE.Camera>}
   */
  loadCamera(cameraIndex) {
    let camera;
    const cameraDef = this.json.cameras[cameraIndex];
    const params = cameraDef[cameraDef.type];
    if (!params) {
      console.warn("THREE.GLTFLoader: Missing camera parameters.");
      return;
    }
    if (cameraDef.type === "perspective") {
      camera = new PerspectiveCamera(MathUtils.radToDeg(params.yfov), params.aspectRatio || 1, params.znear || 1, params.zfar || 2e6);
    } else if (cameraDef.type === "orthographic") {
      camera = new OrthographicCamera(-params.xmag, params.xmag, params.ymag, -params.ymag, params.znear, params.zfar);
    }
    if (cameraDef.name) camera.name = this.createUniqueName(cameraDef.name);
    assignExtrasToUserData(camera, cameraDef);
    return Promise.resolve(camera);
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#skins
   * @param {number} skinIndex
   * @return {Promise<Skeleton>}
   */
  loadSkin(skinIndex) {
    const skinDef = this.json.skins[skinIndex];
    const pending = [];
    for (let i = 0, il = skinDef.joints.length; i < il; i++) {
      pending.push(this._loadNodeShallow(skinDef.joints[i]));
    }
    if (skinDef.inverseBindMatrices !== void 0) {
      pending.push(this.getDependency("accessor", skinDef.inverseBindMatrices));
    } else {
      pending.push(null);
    }
    return Promise.all(pending).then(function(results) {
      const inverseBindMatrices = results.pop();
      const jointNodes = results;
      const bones = [];
      const boneInverses = [];
      for (let i = 0, il = jointNodes.length; i < il; i++) {
        const jointNode = jointNodes[i];
        if (jointNode) {
          bones.push(jointNode);
          const mat = new Matrix4();
          if (inverseBindMatrices !== null) {
            mat.fromArray(inverseBindMatrices.array, i * 16);
          }
          boneInverses.push(mat);
        } else {
          console.warn('THREE.GLTFLoader: Joint "%s" could not be found.', skinDef.joints[i]);
        }
      }
      return new Skeleton(bones, boneInverses);
    });
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#animations
   * @param {number} animationIndex
   * @return {Promise<AnimationClip>}
   */
  loadAnimation(animationIndex) {
    const json = this.json;
    const parser = this;
    const animationDef = json.animations[animationIndex];
    const animationName = animationDef.name ? animationDef.name : "animation_" + animationIndex;
    const pendingNodes = [];
    const pendingInputAccessors = [];
    const pendingOutputAccessors = [];
    const pendingSamplers = [];
    const pendingTargets = [];
    for (let i = 0, il = animationDef.channels.length; i < il; i++) {
      const channel = animationDef.channels[i];
      const sampler = animationDef.samplers[channel.sampler];
      const target = channel.target;
      const name2 = target.node;
      const input = animationDef.parameters !== void 0 ? animationDef.parameters[sampler.input] : sampler.input;
      const output = animationDef.parameters !== void 0 ? animationDef.parameters[sampler.output] : sampler.output;
      if (target.node === void 0) continue;
      pendingNodes.push(this.getDependency("node", name2));
      pendingInputAccessors.push(this.getDependency("accessor", input));
      pendingOutputAccessors.push(this.getDependency("accessor", output));
      pendingSamplers.push(sampler);
      pendingTargets.push(target);
    }
    return Promise.all([
      Promise.all(pendingNodes),
      Promise.all(pendingInputAccessors),
      Promise.all(pendingOutputAccessors),
      Promise.all(pendingSamplers),
      Promise.all(pendingTargets)
    ]).then(function(dependencies) {
      const nodes = dependencies[0];
      const inputAccessors = dependencies[1];
      const outputAccessors = dependencies[2];
      const samplers = dependencies[3];
      const targets = dependencies[4];
      const tracks = [];
      for (let i = 0, il = nodes.length; i < il; i++) {
        const node = nodes[i];
        const inputAccessor = inputAccessors[i];
        const outputAccessor = outputAccessors[i];
        const sampler = samplers[i];
        const target = targets[i];
        if (node === void 0) continue;
        if (node.updateMatrix) {
          node.updateMatrix();
        }
        const createdTracks = parser._createAnimationTracks(node, inputAccessor, outputAccessor, sampler, target);
        if (createdTracks) {
          for (let k = 0; k < createdTracks.length; k++) {
            tracks.push(createdTracks[k]);
          }
        }
      }
      return new AnimationClip(animationName, void 0, tracks);
    });
  }
  createNodeMesh(nodeIndex) {
    const json = this.json;
    const parser = this;
    const nodeDef = json.nodes[nodeIndex];
    if (nodeDef.mesh === void 0) return null;
    return parser.getDependency("mesh", nodeDef.mesh).then(function(mesh) {
      const node = parser._getNodeRef(parser.meshCache, nodeDef.mesh, mesh);
      if (nodeDef.weights !== void 0) {
        node.traverse(function(o) {
          if (!o.isMesh) return;
          for (let i = 0, il = nodeDef.weights.length; i < il; i++) {
            o.morphTargetInfluences[i] = nodeDef.weights[i];
          }
        });
      }
      return node;
    });
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#nodes-and-hierarchy
   * @param {number} nodeIndex
   * @return {Promise<Object3D>}
   */
  loadNode(nodeIndex) {
    const json = this.json;
    const parser = this;
    const nodeDef = json.nodes[nodeIndex];
    const nodePending = parser._loadNodeShallow(nodeIndex);
    const childPending = [];
    const childrenDef = nodeDef.children || [];
    for (let i = 0, il = childrenDef.length; i < il; i++) {
      childPending.push(parser.getDependency("node", childrenDef[i]));
    }
    const skeletonPending = nodeDef.skin === void 0 ? Promise.resolve(null) : parser.getDependency("skin", nodeDef.skin);
    return Promise.all([
      nodePending,
      Promise.all(childPending),
      skeletonPending
    ]).then(function(results) {
      const node = results[0];
      const children = results[1];
      const skeleton = results[2];
      if (skeleton !== null) {
        node.traverse(function(mesh) {
          if (!mesh.isSkinnedMesh) return;
          mesh.bind(skeleton, _identityMatrix);
        });
      }
      for (let i = 0, il = children.length; i < il; i++) {
        node.add(children[i]);
      }
      return node;
    });
  }
  // ._loadNodeShallow() parses a single node.
  // skin and child nodes are created and added in .loadNode() (no '_' prefix).
  _loadNodeShallow(nodeIndex) {
    const json = this.json;
    const extensions = this.extensions;
    const parser = this;
    if (this.nodeCache[nodeIndex] !== void 0) {
      return this.nodeCache[nodeIndex];
    }
    const nodeDef = json.nodes[nodeIndex];
    const nodeName = nodeDef.name ? parser.createUniqueName(nodeDef.name) : "";
    const pending = [];
    const meshPromise = parser._invokeOne(function(ext) {
      return ext.createNodeMesh && ext.createNodeMesh(nodeIndex);
    });
    if (meshPromise) {
      pending.push(meshPromise);
    }
    if (nodeDef.camera !== void 0) {
      pending.push(parser.getDependency("camera", nodeDef.camera).then(function(camera) {
        return parser._getNodeRef(parser.cameraCache, nodeDef.camera, camera);
      }));
    }
    parser._invokeAll(function(ext) {
      return ext.createNodeAttachment && ext.createNodeAttachment(nodeIndex);
    }).forEach(function(promise) {
      pending.push(promise);
    });
    this.nodeCache[nodeIndex] = Promise.all(pending).then(function(objects) {
      let node;
      if (nodeDef.isBone === true) {
        node = new Bone();
      } else if (objects.length > 1) {
        node = new Group();
      } else if (objects.length === 1) {
        node = objects[0];
      } else {
        node = new Object3D();
      }
      if (node !== objects[0]) {
        for (let i = 0, il = objects.length; i < il; i++) {
          node.add(objects[i]);
        }
      }
      if (nodeDef.name) {
        node.userData.name = nodeDef.name;
        node.name = nodeName;
      }
      assignExtrasToUserData(node, nodeDef);
      if (nodeDef.extensions) addUnknownExtensionsToUserData(extensions, node, nodeDef);
      if (nodeDef.matrix !== void 0) {
        const matrix = new Matrix4();
        matrix.fromArray(nodeDef.matrix);
        node.applyMatrix4(matrix);
      } else {
        if (nodeDef.translation !== void 0) {
          node.position.fromArray(nodeDef.translation);
        }
        if (nodeDef.rotation !== void 0) {
          node.quaternion.fromArray(nodeDef.rotation);
        }
        if (nodeDef.scale !== void 0) {
          node.scale.fromArray(nodeDef.scale);
        }
      }
      if (!parser.associations.has(node)) {
        parser.associations.set(node, {});
      }
      parser.associations.get(node).nodes = nodeIndex;
      return node;
    });
    return this.nodeCache[nodeIndex];
  }
  /**
   * Specification: https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#scenes
   * @param {number} sceneIndex
   * @return {Promise<Group>}
   */
  loadScene(sceneIndex) {
    const extensions = this.extensions;
    const sceneDef = this.json.scenes[sceneIndex];
    const parser = this;
    const scene = new Group();
    if (sceneDef.name) scene.name = parser.createUniqueName(sceneDef.name);
    assignExtrasToUserData(scene, sceneDef);
    if (sceneDef.extensions) addUnknownExtensionsToUserData(extensions, scene, sceneDef);
    const nodeIds = sceneDef.nodes || [];
    const pending = [];
    for (let i = 0, il = nodeIds.length; i < il; i++) {
      pending.push(parser.getDependency("node", nodeIds[i]));
    }
    return Promise.all(pending).then(function(nodes) {
      for (let i = 0, il = nodes.length; i < il; i++) {
        scene.add(nodes[i]);
      }
      const reduceAssociations = (node) => {
        const reducedAssociations = /* @__PURE__ */ new Map();
        for (const [key, value] of parser.associations) {
          if (key instanceof Material || key instanceof Texture) {
            reducedAssociations.set(key, value);
          }
        }
        node.traverse((node2) => {
          const mappings = parser.associations.get(node2);
          if (mappings != null) {
            reducedAssociations.set(node2, mappings);
          }
        });
        return reducedAssociations;
      };
      parser.associations = reduceAssociations(scene);
      return scene;
    });
  }
  _createAnimationTracks(node, inputAccessor, outputAccessor, sampler, target) {
    const tracks = [];
    const targetName = node.name ? node.name : node.uuid;
    const targetNames = [];
    if (PATH_PROPERTIES[target.path] === PATH_PROPERTIES.weights) {
      node.traverse(function(object) {
        if (object.morphTargetInfluences) {
          targetNames.push(object.name ? object.name : object.uuid);
        }
      });
    } else {
      targetNames.push(targetName);
    }
    let TypedKeyframeTrack;
    switch (PATH_PROPERTIES[target.path]) {
      case PATH_PROPERTIES.weights:
        TypedKeyframeTrack = NumberKeyframeTrack;
        break;
      case PATH_PROPERTIES.rotation:
        TypedKeyframeTrack = QuaternionKeyframeTrack;
        break;
      case PATH_PROPERTIES.position:
      case PATH_PROPERTIES.scale:
        TypedKeyframeTrack = VectorKeyframeTrack;
        break;
      default:
        switch (outputAccessor.itemSize) {
          case 1:
            TypedKeyframeTrack = NumberKeyframeTrack;
            break;
          case 2:
          case 3:
          default:
            TypedKeyframeTrack = VectorKeyframeTrack;
            break;
        }
        break;
    }
    const interpolation = sampler.interpolation !== void 0 ? INTERPOLATION[sampler.interpolation] : InterpolateLinear;
    const outputArray = this._getArrayFromAccessor(outputAccessor);
    for (let j = 0, jl = targetNames.length; j < jl; j++) {
      const track = new TypedKeyframeTrack(
        targetNames[j] + "." + PATH_PROPERTIES[target.path],
        inputAccessor.array,
        outputArray,
        interpolation
      );
      if (sampler.interpolation === "CUBICSPLINE") {
        this._createCubicSplineTrackInterpolant(track);
      }
      tracks.push(track);
    }
    return tracks;
  }
  _getArrayFromAccessor(accessor) {
    let outputArray = accessor.array;
    if (accessor.normalized) {
      const scale = getNormalizedComponentScale(outputArray.constructor);
      const scaled = new Float32Array(outputArray.length);
      for (let j = 0, jl = outputArray.length; j < jl; j++) {
        scaled[j] = outputArray[j] * scale;
      }
      outputArray = scaled;
    }
    return outputArray;
  }
  _createCubicSplineTrackInterpolant(track) {
    track.createInterpolant = function InterpolantFactoryMethodGLTFCubicSpline(result) {
      const interpolantType = this instanceof QuaternionKeyframeTrack ? GLTFCubicSplineQuaternionInterpolant : GLTFCubicSplineInterpolant;
      return new interpolantType(this.times, this.values, this.getValueSize() / 3, result);
    };
    track.createInterpolant.isInterpolantFactoryMethodGLTFCubicSpline = true;
  }
};
function computeBounds(geometry, primitiveDef, parser) {
  const attributes = primitiveDef.attributes;
  const box = new Box3();
  if (attributes.POSITION !== void 0) {
    const accessor = parser.json.accessors[attributes.POSITION];
    const min = accessor.min;
    const max = accessor.max;
    if (min !== void 0 && max !== void 0) {
      box.set(
        new Vector32(min[0], min[1], min[2]),
        new Vector32(max[0], max[1], max[2])
      );
      if (accessor.normalized) {
        const boxScale = getNormalizedComponentScale(WEBGL_COMPONENT_TYPES[accessor.componentType]);
        box.min.multiplyScalar(boxScale);
        box.max.multiplyScalar(boxScale);
      }
    } else {
      console.warn("THREE.GLTFLoader: Missing min/max properties for accessor POSITION.");
      return;
    }
  } else {
    return;
  }
  const targets = primitiveDef.targets;
  if (targets !== void 0) {
    const maxDisplacement = new Vector32();
    const vector = new Vector32();
    for (let i = 0, il = targets.length; i < il; i++) {
      const target = targets[i];
      if (target.POSITION !== void 0) {
        const accessor = parser.json.accessors[target.POSITION];
        const min = accessor.min;
        const max = accessor.max;
        if (min !== void 0 && max !== void 0) {
          vector.setX(Math.max(Math.abs(min[0]), Math.abs(max[0])));
          vector.setY(Math.max(Math.abs(min[1]), Math.abs(max[1])));
          vector.setZ(Math.max(Math.abs(min[2]), Math.abs(max[2])));
          if (accessor.normalized) {
            const boxScale = getNormalizedComponentScale(WEBGL_COMPONENT_TYPES[accessor.componentType]);
            vector.multiplyScalar(boxScale);
          }
          maxDisplacement.max(vector);
        } else {
          console.warn("THREE.GLTFLoader: Missing min/max properties for accessor POSITION.");
        }
      }
    }
    box.expandByVector(maxDisplacement);
  }
  geometry.boundingBox = box;
  const sphere = new Sphere();
  box.getCenter(sphere.center);
  sphere.radius = box.min.distanceTo(box.max) / 2;
  geometry.boundingSphere = sphere;
}
function addPrimitiveAttributes(geometry, primitiveDef, parser) {
  const attributes = primitiveDef.attributes;
  const pending = [];
  function assignAttributeAccessor(accessorIndex, attributeName) {
    return parser.getDependency("accessor", accessorIndex).then(function(accessor) {
      geometry.setAttribute(attributeName, accessor);
    });
  }
  for (const gltfAttributeName in attributes) {
    const threeAttributeName = ATTRIBUTES[gltfAttributeName] || gltfAttributeName.toLowerCase();
    if (threeAttributeName in geometry.attributes) continue;
    pending.push(assignAttributeAccessor(attributes[gltfAttributeName], threeAttributeName));
  }
  if (primitiveDef.indices !== void 0 && !geometry.index) {
    const accessor = parser.getDependency("accessor", primitiveDef.indices).then(function(accessor2) {
      geometry.setIndex(accessor2);
    });
    pending.push(accessor);
  }
  if (ColorManagement.workingColorSpace !== LinearSRGBColorSpace && "COLOR_0" in attributes) {
    console.warn(`THREE.GLTFLoader: Converting vertex colors from "srgb-linear" to "${ColorManagement.workingColorSpace}" not supported.`);
  }
  assignExtrasToUserData(geometry, primitiveDef);
  computeBounds(geometry, primitiveDef, parser);
  return Promise.all(pending).then(function() {
    return primitiveDef.targets !== void 0 ? addMorphTargets(geometry, primitiveDef.targets, parser) : geometry;
  });
}

// node_modules/3d-tiles-renderer/src/three/loaders/B3DMLoader.js
var B3DMLoader = class extends B3DMLoaderBase {
  constructor(manager = DefaultLoadingManager) {
    super();
    this.manager = manager;
    this.adjustmentTransform = new Matrix42();
  }
  parse(buffer) {
    const b3dm = super.parse(buffer);
    const gltfBuffer = b3dm.glbBytes.slice().buffer;
    return new Promise((resolve, reject) => {
      const manager = this.manager;
      const fetchOptions = this.fetchOptions;
      const loader = manager.getHandler("path.gltf") || new GLTFLoader(manager);
      if (fetchOptions.credentials === "include" && fetchOptions.mode === "cors") {
        loader.setCrossOrigin("use-credentials");
      }
      if ("credentials" in fetchOptions) {
        loader.setWithCredentials(fetchOptions.credentials === "include");
      }
      if (fetchOptions.headers) {
        loader.setRequestHeader(fetchOptions.headers);
      }
      let workingPath = this.workingPath;
      if (!/[\\/]$/.test(workingPath) && workingPath.length) {
        workingPath += "/";
      }
      const adjustmentTransform = this.adjustmentTransform;
      loader.parse(gltfBuffer, workingPath, (model) => {
        const { batchTable, featureTable } = b3dm;
        const { scene } = model;
        const rtcCenter = featureTable.getData("RTC_CENTER");
        if (rtcCenter) {
          scene.position.x += rtcCenter[0];
          scene.position.y += rtcCenter[1];
          scene.position.z += rtcCenter[2];
        }
        model.scene.updateMatrix();
        model.scene.matrix.multiply(adjustmentTransform);
        model.scene.matrix.decompose(model.scene.position, model.scene.quaternion, model.scene.scale);
        model.batchTable = batchTable;
        model.featureTable = featureTable;
        scene.batchTable = batchTable;
        scene.featureTable = featureTable;
        resolve(model);
      }, reject);
    });
  }
};

// node_modules/3d-tiles-renderer/src/base/loaders/PNTSLoaderBase.js
var PNTSLoaderBase = class extends LoaderBase {
  parse(buffer) {
    const dataView = new DataView(buffer);
    const magic = readMagicBytes(dataView);
    console.assert(magic === "pnts");
    const version = dataView.getUint32(4, true);
    console.assert(version === 1);
    const byteLength = dataView.getUint32(8, true);
    console.assert(byteLength === buffer.byteLength);
    const featureTableJSONByteLength = dataView.getUint32(12, true);
    const featureTableBinaryByteLength = dataView.getUint32(16, true);
    const batchTableJSONByteLength = dataView.getUint32(20, true);
    const batchTableBinaryByteLength = dataView.getUint32(24, true);
    const featureTableStart = 28;
    const featureTableBuffer = buffer.slice(
      featureTableStart,
      featureTableStart + featureTableJSONByteLength + featureTableBinaryByteLength
    );
    const featureTable = new FeatureTable(
      featureTableBuffer,
      0,
      featureTableJSONByteLength,
      featureTableBinaryByteLength
    );
    const batchTableStart = featureTableStart + featureTableJSONByteLength + featureTableBinaryByteLength;
    const batchTableBuffer = buffer.slice(
      batchTableStart,
      batchTableStart + batchTableJSONByteLength + batchTableBinaryByteLength
    );
    const batchTable = new BatchTable(
      batchTableBuffer,
      featureTable.getData("BATCH_LENGTH") || featureTable.getData("POINTS_LENGTH"),
      0,
      batchTableJSONByteLength,
      batchTableBinaryByteLength
    );
    return Promise.resolve({
      version,
      featureTable,
      batchTable
    });
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/PNTSLoader.js
import {
  Points as Points2,
  PointsMaterial as PointsMaterial2,
  BufferGeometry as BufferGeometry3,
  BufferAttribute as BufferAttribute3,
  DefaultLoadingManager as DefaultLoadingManager2,
  Vector3 as Vector33,
  Color as Color2
} from "three";

// node_modules/3d-tiles-renderer/src/utilities/rgb565torgb.js
function rgb565torgb(rgb565) {
  const red5 = rgb565 >> 11;
  const green6 = rgb565 >> 5 & 63;
  const blue5 = rgb565 & 31;
  const red8 = Math.round(red5 / 31 * 255);
  const green8 = Math.round(green6 / 63 * 255);
  const blue8 = Math.round(blue5 / 31 * 255);
  return [red8, green8, blue8];
}

// node_modules/3d-tiles-renderer/src/three/loaders/PNTSLoader.js
var DRACO_ATTRIBUTE_MAP = {
  RGB: "color",
  POSITION: "position"
};
var PNTSLoader = class extends PNTSLoaderBase {
  constructor(manager = DefaultLoadingManager2) {
    super();
    this.manager = manager;
  }
  parse(buffer) {
    return super.parse(buffer).then(async (result) => {
      const { featureTable, batchTable } = result;
      const material = new PointsMaterial2();
      const extensions = featureTable.header.extensions;
      const translationOffset = new Vector33();
      let geometry;
      if (extensions && extensions["3DTILES_draco_point_compression"]) {
        const { byteOffset, byteLength, properties } = extensions["3DTILES_draco_point_compression"];
        const dracoLoader = this.manager.getHandler("draco.drc");
        if (dracoLoader == null) {
          throw new Error("PNTSLoader: dracoLoader not available.");
        }
        const attributeIDs = {};
        for (const key in properties) {
          if (key in DRACO_ATTRIBUTE_MAP && key in properties) {
            const mappedKey = DRACO_ATTRIBUTE_MAP[key];
            attributeIDs[mappedKey] = properties[key];
          }
        }
        const taskConfig = {
          attributeIDs,
          attributeTypes: {
            position: "Float32Array",
            color: "Uint8Array"
          },
          useUniqueIDs: true
        };
        const buffer2 = featureTable.getBuffer(byteOffset, byteLength);
        geometry = await dracoLoader.decodeGeometry(buffer2, taskConfig);
        if (geometry.attributes.color) {
          material.vertexColors = true;
        }
      } else {
        const POINTS_LENGTH = featureTable.getData("POINTS_LENGTH");
        const POSITION = featureTable.getData("POSITION", POINTS_LENGTH, "FLOAT", "VEC3");
        const RGB = featureTable.getData("RGB", POINTS_LENGTH, "UNSIGNED_BYTE", "VEC3");
        const RGBA = featureTable.getData("RGBA", POINTS_LENGTH, "UNSIGNED_BYTE", "VEC4");
        const RGB565 = featureTable.getData("RGB565", POINTS_LENGTH, "UNSIGNED_SHORT", "SCALAR");
        const CONSTANT_RGBA = featureTable.getData("CONSTANT_RGBA", POINTS_LENGTH, "UNSIGNED_BYTE", "VEC4");
        const POSITION_QUANTIZED = featureTable.getData("POSITION_QUANTIZED", POINTS_LENGTH, "UNSIGNED_SHORT", "VEC3");
        const QUANTIZED_VOLUME_SCALE = featureTable.getData("QUANTIZED_VOLUME_SCALE", POINTS_LENGTH, "FLOAT", "VEC3");
        const QUANTIZED_VOLUME_OFFSET = featureTable.getData("QUANTIZED_VOLUME_OFFSET", POINTS_LENGTH, "FLOAT", "VEC3");
        geometry = new BufferGeometry3();
        if (POSITION_QUANTIZED) {
          const decodedPositions = new Float32Array(POINTS_LENGTH * 3);
          for (let i = 0; i < POINTS_LENGTH; i++) {
            for (let j = 0; j < 3; j++) {
              const index = 3 * i + j;
              decodedPositions[index] = POSITION_QUANTIZED[index] / 65535 * QUANTIZED_VOLUME_SCALE[j];
            }
          }
          translationOffset.x = QUANTIZED_VOLUME_OFFSET[0];
          translationOffset.y = QUANTIZED_VOLUME_OFFSET[1];
          translationOffset.z = QUANTIZED_VOLUME_OFFSET[2];
          geometry.setAttribute("position", new BufferAttribute3(decodedPositions, 3, false));
        } else {
          geometry.setAttribute("position", new BufferAttribute3(POSITION, 3, false));
        }
        if (RGBA !== null) {
          geometry.setAttribute("color", new BufferAttribute3(RGBA, 4, true));
          material.vertexColors = true;
          material.transparent = true;
          material.depthWrite = false;
        } else if (RGB !== null) {
          geometry.setAttribute("color", new BufferAttribute3(RGB, 3, true));
          material.vertexColors = true;
        } else if (RGB565 !== null) {
          const color = new Uint8Array(POINTS_LENGTH * 3);
          for (let i = 0; i < POINTS_LENGTH; i++) {
            const rgbColor = rgb565torgb(RGB565[i]);
            for (let j = 0; j < 3; j++) {
              const index = 3 * i + j;
              color[index] = rgbColor[j];
            }
          }
          geometry.setAttribute("color", new BufferAttribute3(color, 3, true));
          material.vertexColors = true;
        } else if (CONSTANT_RGBA !== null) {
          const color = new Color2(CONSTANT_RGBA[0], CONSTANT_RGBA[1], CONSTANT_RGBA[2]);
          material.color = color;
          const opacity = CONSTANT_RGBA[3] / 255;
          if (opacity < 1) {
            material.opacity = opacity;
            material.transparent = true;
            material.depthWrite = false;
          }
        }
      }
      [
        "BATCH_LENGTH",
        "NORMAL",
        "NORMAL_OCT16P"
      ].forEach((feature) => {
        if (feature in featureTable.header) {
          console.warn(
            `PNTSLoader: Unsupported FeatureTable feature "${feature}" detected.`
          );
        }
      });
      const object = new Points2(geometry, material);
      object.position.copy(translationOffset);
      result.scene = object;
      result.scene.featureTable = featureTable;
      result.scene.batchTable = batchTable;
      const rtcCenter = featureTable.getData("RTC_CENTER");
      if (rtcCenter) {
        result.scene.position.x += rtcCenter[0];
        result.scene.position.y += rtcCenter[1];
        result.scene.position.z += rtcCenter[2];
      }
      return result;
    });
  }
};

// node_modules/3d-tiles-renderer/src/base/loaders/I3DMLoaderBase.js
var I3DMLoaderBase = class extends LoaderBase {
  parse(buffer) {
    const dataView = new DataView(buffer);
    const magic = readMagicBytes(dataView);
    console.assert(magic === "i3dm");
    const version = dataView.getUint32(4, true);
    console.assert(version === 1);
    const byteLength = dataView.getUint32(8, true);
    console.assert(byteLength === buffer.byteLength);
    const featureTableJSONByteLength = dataView.getUint32(12, true);
    const featureTableBinaryByteLength = dataView.getUint32(16, true);
    const batchTableJSONByteLength = dataView.getUint32(20, true);
    const batchTableBinaryByteLength = dataView.getUint32(24, true);
    const gltfFormat = dataView.getUint32(28, true);
    const featureTableStart = 32;
    const featureTableBuffer = buffer.slice(
      featureTableStart,
      featureTableStart + featureTableJSONByteLength + featureTableBinaryByteLength
    );
    const featureTable = new FeatureTable(
      featureTableBuffer,
      0,
      featureTableJSONByteLength,
      featureTableBinaryByteLength
    );
    const batchTableStart = featureTableStart + featureTableJSONByteLength + featureTableBinaryByteLength;
    const batchTableBuffer = buffer.slice(
      batchTableStart,
      batchTableStart + batchTableJSONByteLength + batchTableBinaryByteLength
    );
    const batchTable = new BatchTable(
      batchTableBuffer,
      featureTable.getData("INSTANCES_LENGTH"),
      0,
      batchTableJSONByteLength,
      batchTableBinaryByteLength
    );
    const glbStart = batchTableStart + batchTableJSONByteLength + batchTableBinaryByteLength;
    const bodyBytes = new Uint8Array(buffer, glbStart, byteLength - glbStart);
    let glbBytes = null;
    let promise = null;
    let gltfWorkingPath = null;
    if (gltfFormat) {
      glbBytes = bodyBytes;
      promise = Promise.resolve();
    } else {
      const externalUri = this.resolveExternalURL(arrayToString(bodyBytes));
      const uriSplits = externalUri.split(/[\\/]/g);
      uriSplits.pop();
      gltfWorkingPath = uriSplits.join("/");
      promise = fetch(externalUri, this.fetchOptions).then((res) => {
        if (!res.ok) {
          throw new Error(`I3DMLoaderBase : Failed to load file "${externalUri}" with status ${res.status} : ${res.statusText}`);
        }
        return res.arrayBuffer();
      }).then((buffer2) => {
        glbBytes = new Uint8Array(buffer2);
      });
    }
    return promise.then(() => {
      return {
        version,
        featureTable,
        batchTable,
        glbBytes,
        gltfWorkingPath
      };
    });
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/I3DMLoader.js
import { DefaultLoadingManager as DefaultLoadingManager3, Matrix4 as Matrix44, InstancedMesh as InstancedMesh2, Vector3 as Vector36, Quaternion as Quaternion2 } from "three";

// node_modules/3d-tiles-renderer/src/three/math/Ellipsoid.js
import { Vector3 as Vector35, Spherical as Spherical2, MathUtils as MathUtils3, Ray, Matrix4 as Matrix43, Sphere as Sphere2, Euler } from "three";

// node_modules/3d-tiles-renderer/src/three/math/GeoUtils.js
import { Spherical, Vector3 as Vector34, MathUtils as MathUtils2 } from "three";
var _spherical = new Spherical();
var _vec = new Vector34();
function swapToGeoFrame(target) {
  const { x, y, z } = target;
  target.x = z;
  target.y = x;
  target.z = y;
}
function latitudeToSphericalPhi(latitude) {
  return -latitude + Math.PI / 2;
}

// node_modules/3d-tiles-renderer/src/three/math/Ellipsoid.js
var _spherical2 = new Spherical2();
var _norm = new Vector35();
var _vec2 = new Vector35();
var _vec22 = new Vector35();
var _vec3 = new Vector35();
var _matrix = new Matrix43();
var _matrix2 = new Matrix43();
var _sphere = new Sphere2();
var _euler = new Euler();
var _vecX = new Vector35();
var _vecY = new Vector35();
var _vecZ = new Vector35();
var _pos = new Vector35();
var _ray = new Ray();
var EPSILON12 = 1e-12;
var CENTER_EPS = 0.1;
var ENU_FRAME = 0;
var CAMERA_FRAME = 1;
var OBJECT_FRAME = 2;
var Ellipsoid = class {
  constructor(x = 1, y = 1, z = 1) {
    this.radius = new Vector35(x, y, z);
  }
  intersectRay(ray, target) {
    _matrix.makeScale(...this.radius).invert();
    _sphere.center.set(0, 0, 0);
    _sphere.radius = 1;
    _ray.copy(ray).applyMatrix4(_matrix);
    if (_ray.intersectSphere(_sphere, target)) {
      _matrix.makeScale(...this.radius);
      target.applyMatrix4(_matrix);
      return target;
    } else {
      return null;
    }
  }
  // returns a frame with Z indicating altitude
  // Y pointing north
  // X pointing east
  getEastNorthUpFrame(lat, lon, target) {
    this.getEastNorthUpAxes(lat, lon, _vecX, _vecY, _vecZ, _pos);
    return target.makeBasis(_vecX, _vecY, _vecZ).setPosition(_pos);
  }
  getEastNorthUpAxes(lat, lon, vecEast, vecNorth, vecUp, point = _pos) {
    this.getCartographicToPosition(lat, lon, 0, point);
    this.getCartographicToNormal(lat, lon, vecUp);
    vecEast.set(-point.y, point.x, 0).normalize();
    vecNorth.crossVectors(vecUp, vecEast).normalize();
  }
  getNorthernTangent(lat, lon, target, westTarget = _vec3) {
    console.log("Ellipsoid: getNorthernTangent has been deprecated. Use getEastNorthUpAxes instead.");
    this.getEastNorthUpAxes(lat, lon, westTarget, target, _vecZ);
    westTarget.multiplyScalar(-1);
    return target;
  }
  // azimuth: measured off of true north, increasing towards "east"
  // elevation: measured off of the horizon, increasing towards sky
  // roll: rotation around northern axis
  getAzElRollFromRotationMatrix(lat, lon, rotationMatrix, target, frame = ENU_FRAME) {
    if (frame === CAMERA_FRAME) {
      _euler.set(-Math.PI / 2, 0, 0, "XYZ");
      _matrix2.makeRotationFromEuler(_euler).premultiply(rotationMatrix);
    } else if (frame === OBJECT_FRAME) {
      _euler.set(-Math.PI / 2, 0, Math.PI, "XYZ");
      _matrix2.makeRotationFromEuler(_euler).premultiply(rotationMatrix);
    } else {
      _matrix2.copy(rotationMatrix);
    }
    this.getEastNorthUpFrame(lat, lon, _matrix).invert();
    _matrix2.premultiply(_matrix);
    _euler.setFromRotationMatrix(_matrix2, "ZXY");
    target.azimuth = -_euler.z;
    target.elevation = _euler.x;
    target.roll = _euler.y;
    return target;
  }
  getRotationMatrixFromAzElRoll(lat, lon, az, el, roll, target, frame = ENU_FRAME) {
    this.getEastNorthUpFrame(lat, lon, _matrix);
    _euler.set(el, roll, -az, "ZXY");
    target.makeRotationFromEuler(_euler).premultiply(_matrix).setPosition(0, 0, 0);
    if (frame === CAMERA_FRAME) {
      _euler.set(Math.PI / 2, 0, 0, "XYZ");
      _matrix2.makeRotationFromEuler(_euler);
      target.multiply(_matrix2);
    } else if (frame === OBJECT_FRAME) {
      _euler.set(-Math.PI / 2, 0, Math.PI, "XYZ");
      _matrix2.makeRotationFromEuler(_euler);
      target.multiply(_matrix2);
    }
    return target;
  }
  getCartographicToPosition(lat, lon, height, target) {
    this.getCartographicToNormal(lat, lon, _norm);
    const radius = this.radius;
    _vec2.copy(_norm);
    _vec2.x *= radius.x ** 2;
    _vec2.y *= radius.y ** 2;
    _vec2.z *= radius.z ** 2;
    const gamma = Math.sqrt(_norm.dot(_vec2));
    _vec2.divideScalar(gamma);
    return target.copy(_vec2).addScaledVector(_norm, height);
  }
  getPositionToCartographic(pos, target) {
    this.getPositionToSurfacePoint(pos, _vec2);
    this.getPositionToNormal(pos, _norm);
    const heightDelta = _vec22.subVectors(pos, _vec2);
    target.lon = Math.atan2(_norm.y, _norm.x);
    target.lat = Math.asin(_norm.z);
    target.height = Math.sign(heightDelta.dot(pos)) * heightDelta.length();
    return target;
  }
  getCartographicToNormal(lat, lon, target) {
    _spherical2.set(1, latitudeToSphericalPhi(lat), lon);
    target.setFromSpherical(_spherical2).normalize();
    swapToGeoFrame(target);
    return target;
  }
  getPositionToNormal(pos, target) {
    const radius = this.radius;
    target.copy(pos);
    target.x /= radius.x ** 2;
    target.y /= radius.y ** 2;
    target.z /= radius.z ** 2;
    target.normalize();
    return target;
  }
  getPositionToSurfacePoint(pos, target) {
    const radius = this.radius;
    const invRadiusSqX = 1 / radius.x ** 2;
    const invRadiusSqY = 1 / radius.y ** 2;
    const invRadiusSqZ = 1 / radius.z ** 2;
    const x2 = pos.x * pos.x * invRadiusSqX;
    const y2 = pos.y * pos.y * invRadiusSqY;
    const z2 = pos.z * pos.z * invRadiusSqZ;
    const squaredNorm = x2 + y2 + z2;
    const ratio = Math.sqrt(1 / squaredNorm);
    const intersection = _vec2.copy(pos).multiplyScalar(ratio);
    if (squaredNorm < CENTER_EPS) {
      return !isFinite(ratio) ? null : target.copy(intersection);
    }
    const gradient = _vec22.set(
      intersection.x * invRadiusSqX * 2,
      intersection.y * invRadiusSqY * 2,
      intersection.z * invRadiusSqZ * 2
    );
    let lambda = (1 - ratio) * pos.length() / (0.5 * gradient.length());
    let correction = 0;
    let func, denominator;
    let xMultiplier, yMultiplier, zMultiplier;
    let xMultiplier2, yMultiplier2, zMultiplier2;
    let xMultiplier3, yMultiplier3, zMultiplier3;
    do {
      lambda -= correction;
      xMultiplier = 1 / (1 + lambda * invRadiusSqX);
      yMultiplier = 1 / (1 + lambda * invRadiusSqY);
      zMultiplier = 1 / (1 + lambda * invRadiusSqZ);
      xMultiplier2 = xMultiplier * xMultiplier;
      yMultiplier2 = yMultiplier * yMultiplier;
      zMultiplier2 = zMultiplier * zMultiplier;
      xMultiplier3 = xMultiplier2 * xMultiplier;
      yMultiplier3 = yMultiplier2 * yMultiplier;
      zMultiplier3 = zMultiplier2 * zMultiplier;
      func = x2 * xMultiplier2 + y2 * yMultiplier2 + z2 * zMultiplier2 - 1;
      denominator = x2 * xMultiplier3 * invRadiusSqX + y2 * yMultiplier3 * invRadiusSqY + z2 * zMultiplier3 * invRadiusSqZ;
      const derivative = -2 * denominator;
      correction = func / derivative;
    } while (Math.abs(func) > EPSILON12);
    return target.set(
      pos.x * xMultiplier,
      pos.y * yMultiplier,
      pos.z * zMultiplier
    );
  }
  calculateHorizonDistance(latitude, elevation) {
    const effectiveRadius = this.calculateEffectiveRadius(latitude);
    return Math.sqrt(2 * effectiveRadius * elevation + elevation ** 2);
  }
  calculateEffectiveRadius(latitude) {
    const semiMajorAxis = this.radius.x;
    const semiMinorAxis = this.radius.z;
    const eSquared = 1 - semiMinorAxis ** 2 / semiMajorAxis ** 2;
    const phi = latitude * MathUtils3.DEG2RAD;
    const sinPhiSquared = Math.sin(phi) ** 2;
    const N = semiMajorAxis / Math.sqrt(1 - eSquared * sinPhiSquared);
    return N;
  }
  getPositionElevation(pos) {
    this.getPositionToSurfacePoint(pos, _vec2);
    const heightDelta = _vec22.subVectors(pos, _vec2);
    return Math.sign(heightDelta.dot(pos)) * heightDelta.length();
  }
  copy(source) {
    this.radius.copy(source.radius);
    return this;
  }
  clone() {
    return new this.constructor().copy(this);
  }
};

// node_modules/3d-tiles-renderer/src/three/math/GeoConstants.js
var WGS84_ELLIPSOID = new Ellipsoid(WGS84_RADIUS, WGS84_RADIUS, WGS84_HEIGHT);
var LUNAR_ELLIPSOID = new Ellipsoid(LUNAR_RADIUS, LUNAR_RADIUS, LUNAR_HEIGHT);

// node_modules/3d-tiles-renderer/src/three/loaders/I3DMLoader.js
var tempFwd = /* @__PURE__ */ new Vector36();
var tempUp = /* @__PURE__ */ new Vector36();
var tempRight = /* @__PURE__ */ new Vector36();
var tempPos = /* @__PURE__ */ new Vector36();
var tempQuat = /* @__PURE__ */ new Quaternion2();
var tempSca = /* @__PURE__ */ new Vector36();
var tempMat = /* @__PURE__ */ new Matrix44();
var tempMat2 = /* @__PURE__ */ new Matrix44();
var tempGlobePos = /* @__PURE__ */ new Vector36();
var tempEnuFrame = /* @__PURE__ */ new Matrix44();
var tempLocalQuat = /* @__PURE__ */ new Quaternion2();
var tempLatLon = {};
var I3DMLoader = class extends I3DMLoaderBase {
  constructor(manager = DefaultLoadingManager3) {
    super();
    this.manager = manager;
    this.adjustmentTransform = new Matrix44();
    this.ellipsoid = WGS84_ELLIPSOID.clone();
  }
  resolveExternalURL(url) {
    return this.manager.resolveURL(super.resolveExternalURL(url));
  }
  parse(buffer) {
    return super.parse(buffer).then((i3dm) => {
      const { featureTable, batchTable } = i3dm;
      const gltfBuffer = i3dm.glbBytes.slice().buffer;
      return new Promise((resolve, reject) => {
        const fetchOptions = this.fetchOptions;
        const manager = this.manager;
        const loader = manager.getHandler("path.gltf") || new GLTFLoader(manager);
        if (fetchOptions.credentials === "include" && fetchOptions.mode === "cors") {
          loader.setCrossOrigin("use-credentials");
        }
        if ("credentials" in fetchOptions) {
          loader.setWithCredentials(fetchOptions.credentials === "include");
        }
        if (fetchOptions.headers) {
          loader.setRequestHeader(fetchOptions.headers);
        }
        let workingPath = i3dm.gltfWorkingPath ?? this.workingPath;
        if (!/[\\/]$/.test(workingPath)) {
          workingPath += "/";
        }
        const adjustmentTransform = this.adjustmentTransform;
        loader.parse(gltfBuffer, workingPath, (model) => {
          const INSTANCES_LENGTH = featureTable.getData("INSTANCES_LENGTH");
          const POSITION = featureTable.getData("POSITION", INSTANCES_LENGTH, "FLOAT", "VEC3");
          const NORMAL_UP = featureTable.getData("NORMAL_UP", INSTANCES_LENGTH, "FLOAT", "VEC3");
          const NORMAL_RIGHT = featureTable.getData("NORMAL_RIGHT", INSTANCES_LENGTH, "FLOAT", "VEC3");
          const SCALE_NON_UNIFORM = featureTable.getData("SCALE_NON_UNIFORM", INSTANCES_LENGTH, "FLOAT", "VEC3");
          const SCALE = featureTable.getData("SCALE", INSTANCES_LENGTH, "FLOAT", "SCALAR");
          const RTC_CENTER = featureTable.getData("RTC_CENTER");
          const EAST_NORTH_UP = featureTable.getData("EAST_NORTH_UP");
          [
            "QUANTIZED_VOLUME_OFFSET",
            "QUANTIZED_VOLUME_SCALE",
            "POSITION_QUANTIZED",
            "NORMAL_UP_OCT32P",
            "NORMAL_RIGHT_OCT32P"
          ].forEach((feature) => {
            if (feature in featureTable.header) {
              console.warn(`I3DMLoader: Unsupported FeatureTable feature "${feature}" detected.`);
            }
          });
          const averageVector = new Vector36();
          for (let i = 0; i < INSTANCES_LENGTH; i++) {
            averageVector.x += POSITION[i * 3 + 0] / INSTANCES_LENGTH;
            averageVector.y += POSITION[i * 3 + 1] / INSTANCES_LENGTH;
            averageVector.z += POSITION[i * 3 + 2] / INSTANCES_LENGTH;
          }
          const instances = [];
          const meshes = [];
          model.scene.updateMatrixWorld();
          model.scene.traverse((child) => {
            if (child.isMesh) {
              meshes.push(child);
              const { geometry, material } = child;
              const instancedMesh = new InstancedMesh2(geometry, material, INSTANCES_LENGTH);
              instancedMesh.position.copy(averageVector);
              if (RTC_CENTER) {
                instancedMesh.position.x += RTC_CENTER[0];
                instancedMesh.position.y += RTC_CENTER[1];
                instancedMesh.position.z += RTC_CENTER[2];
              }
              instances.push(instancedMesh);
            }
          });
          for (let i = 0; i < INSTANCES_LENGTH; i++) {
            tempPos.set(
              POSITION[i * 3 + 0] - averageVector.x,
              POSITION[i * 3 + 1] - averageVector.y,
              POSITION[i * 3 + 2] - averageVector.z
            );
            tempQuat.identity();
            if (NORMAL_UP) {
              tempUp.set(
                NORMAL_UP[i * 3 + 0],
                NORMAL_UP[i * 3 + 1],
                NORMAL_UP[i * 3 + 2]
              );
              tempRight.set(
                NORMAL_RIGHT[i * 3 + 0],
                NORMAL_RIGHT[i * 3 + 1],
                NORMAL_RIGHT[i * 3 + 2]
              );
              tempFwd.crossVectors(tempRight, tempUp).normalize();
              tempMat.makeBasis(
                tempRight,
                tempUp,
                tempFwd
              );
              tempQuat.setFromRotationMatrix(tempMat);
            }
            tempSca.set(1, 1, 1);
            if (SCALE_NON_UNIFORM) {
              tempSca.set(
                SCALE_NON_UNIFORM[i * 3 + 0],
                SCALE_NON_UNIFORM[i * 3 + 1],
                SCALE_NON_UNIFORM[i * 3 + 2]
              );
            }
            if (SCALE) {
              tempSca.multiplyScalar(SCALE[i]);
            }
            for (let j = 0, l = instances.length; j < l; j++) {
              const instance = instances[j];
              tempLocalQuat.copy(tempQuat);
              if (EAST_NORTH_UP) {
                instance.updateMatrixWorld();
                tempGlobePos.copy(tempPos).applyMatrix4(instance.matrixWorld);
                this.ellipsoid.getPositionToCartographic(tempGlobePos, tempLatLon);
                this.ellipsoid.getEastNorthUpFrame(tempLatLon.lat, tempLatLon.lon, tempEnuFrame);
                tempLocalQuat.setFromRotationMatrix(tempEnuFrame);
              }
              tempMat.compose(tempPos, tempLocalQuat, tempSca).multiply(adjustmentTransform);
              const mesh = meshes[j];
              tempMat2.multiplyMatrices(tempMat, mesh.matrixWorld);
              instance.setMatrixAt(i, tempMat2);
            }
          }
          model.scene.clear();
          model.scene.add(...instances);
          model.batchTable = batchTable;
          model.featureTable = featureTable;
          model.scene.batchTable = batchTable;
          model.scene.featureTable = featureTable;
          resolve(model);
        }, reject);
      });
    });
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/CMPTLoader.js
import { Group as Group2, DefaultLoadingManager as DefaultLoadingManager4, Matrix4 as Matrix45 } from "three";

// node_modules/3d-tiles-renderer/src/base/loaders/CMPTLoaderBase.js
var CMPTLoaderBase = class extends LoaderBase {
  parse(buffer) {
    const dataView = new DataView(buffer);
    const magic = readMagicBytes(dataView);
    console.assert(magic === "cmpt", 'CMPTLoader: The magic bytes equal "cmpt".');
    const version = dataView.getUint32(4, true);
    console.assert(version === 1, 'CMPTLoader: The version listed in the header is "1".');
    const byteLength = dataView.getUint32(8, true);
    console.assert(byteLength === buffer.byteLength, "CMPTLoader: The contents buffer length listed in the header matches the file.");
    const tilesLength = dataView.getUint32(12, true);
    const tiles = [];
    let offset = 16;
    for (let i = 0; i < tilesLength; i++) {
      const tileView = new DataView(buffer, offset, 12);
      const tileMagic = readMagicBytes(tileView);
      const tileVersion = tileView.getUint32(4, true);
      const byteLength2 = tileView.getUint32(8, true);
      const tileBuffer = new Uint8Array(buffer, offset, byteLength2);
      tiles.push({
        type: tileMagic,
        buffer: tileBuffer,
        version: tileVersion
      });
      offset += byteLength2;
    }
    return {
      version,
      tiles
    };
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/CMPTLoader.js
var CMPTLoader = class extends CMPTLoaderBase {
  constructor(manager = DefaultLoadingManager4) {
    super();
    this.manager = manager;
    this.adjustmentTransform = new Matrix45();
    this.ellipsoid = WGS84_ELLIPSOID.clone();
  }
  parse(buffer) {
    const result = super.parse(buffer);
    const { manager, ellipsoid, adjustmentTransform } = this;
    const promises = [];
    for (const i in result.tiles) {
      const { type, buffer: buffer2 } = result.tiles[i];
      switch (type) {
        case "b3dm": {
          const slicedBuffer = buffer2.slice();
          const loader = new B3DMLoader(manager);
          loader.workingPath = this.workingPath;
          loader.fetchOptions = this.fetchOptions;
          loader.adjustmentTransform.copy(adjustmentTransform);
          const promise = loader.parse(slicedBuffer.buffer);
          promises.push(promise);
          break;
        }
        case "pnts": {
          const slicedBuffer = buffer2.slice();
          const loader = new PNTSLoader(manager);
          loader.workingPath = this.workingPath;
          loader.fetchOptions = this.fetchOptions;
          const promise = loader.parse(slicedBuffer.buffer);
          promises.push(promise);
          break;
        }
        case "i3dm": {
          const slicedBuffer = buffer2.slice();
          const loader = new I3DMLoader(manager);
          loader.workingPath = this.workingPath;
          loader.fetchOptions = this.fetchOptions;
          loader.ellipsoid.copy(ellipsoid);
          loader.adjustmentTransform.copy(adjustmentTransform);
          const promise = loader.parse(slicedBuffer.buffer);
          promises.push(promise);
          break;
        }
      }
    }
    return Promise.all(promises).then((results) => {
      const group = new Group2();
      results.forEach((result2) => {
        group.add(result2.scene);
      });
      return {
        tiles: results,
        scene: group
      };
    });
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/GLTFExtensionLoader.js
import { DefaultLoadingManager as DefaultLoadingManager5 } from "three";

// node_modules/3d-tiles-renderer/src/plugins/three/GoogleAttributionsManager.js
var GoogleAttributionsManager = class {
  constructor() {
    this.creditsCount = {};
  }
  _adjustAttributions(line, add) {
    const creditsCount = this.creditsCount;
    const tokens = line.split(/;/g);
    for (let i = 0, l = tokens.length; i < l; i++) {
      const t = tokens[i];
      if (!(t in creditsCount)) {
        creditsCount[t] = 0;
      }
      creditsCount[t] += add ? 1 : -1;
      if (creditsCount[t] <= 0) {
        delete creditsCount[t];
      }
    }
  }
  addAttributions(line) {
    this._adjustAttributions(line, true);
  }
  removeAttributions(line) {
    this._adjustAttributions(line, false);
  }
  toString() {
    const sortedByCount = Object.entries(this.creditsCount).sort((a, b) => {
      const countA = a[1];
      const countB = b[1];
      return countB - countA;
    });
    return sortedByCount.map((pair) => pair[0]).join("; ");
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/GoogleCloudAuthPlugin.js
function getSessionToken(root) {
  let sessionToken = null;
  traverseSet(root, (tile) => {
    if (tile.content && tile.content.uri) {
      const [, params] = tile.content.uri.split("?");
      sessionToken = new URLSearchParams(params).get("session");
      return true;
    }
    return false;
  });
  return sessionToken;
}
var GoogleCloudAuthPlugin = class {
  constructor({ apiToken, autoRefreshToken = false, logoUrl = null, useRecommendedSettings = true }) {
    this.name = "GOOGLE_CLOUD_AUTH_PLUGIN";
    this.apiToken = apiToken;
    this.autoRefreshToken = autoRefreshToken;
    this.useRecommendedSettings = useRecommendedSettings;
    this.logoUrl = logoUrl;
    this.sessionToken = null;
    this.tiles = null;
    this._onLoadCallback = null;
    this._visibilityChangeCallback = null;
    this._tokenRefreshPromise = null;
    this._attributionsManager = new GoogleAttributionsManager();
    this._logoAttribution = {
      value: "",
      type: "image",
      collapsible: false
    };
    this._attribution = {
      value: "",
      type: "string",
      collapsible: true
    };
  }
  init(tiles) {
    if (tiles == null) {
      return;
    }
    tiles.resetFailedTiles();
    if (tiles.rootURL == null) {
      tiles.rootURL = "https://tile.googleapis.com/v1/3dtiles/root.json";
    }
    if (this.useRecommendedSettings) {
      tiles.parseQueue.maxJobs = 10;
      tiles.downloadQueue.maxJobs = 30;
      tiles.errorTarget = 40;
    }
    this.tiles = tiles;
    this._onLoadCallback = ({ tileSet }) => {
      this.sessionToken = getSessionToken(tileSet.root);
      tiles.removeEventListener("load-tile-set", this._onLoadCallback);
    };
    this._visibilityChangeCallback = ({ tile, visible }) => {
      const copyright = tile.cached.metadata.asset.copyright || "";
      if (visible) {
        this._attributionsManager.addAttributions(copyright);
      } else {
        this._attributionsManager.removeAttributions(copyright);
      }
    };
    tiles.addEventListener("load-tile-set", this._onLoadCallback);
    tiles.addEventListener("tile-visibility-change", this._visibilityChangeCallback);
  }
  getAttributions(target) {
    if (this.tiles.visibleTiles.size > 0) {
      if (this.logoUrl) {
        this._logoAttribution.value = this.logoUrl;
        target.push(this._logoAttribution);
      }
      this._attribution.value = this._attributionsManager.toString();
      target.push(this._attribution);
    }
  }
  preprocessURL(uri) {
    uri = new URL(uri);
    if (/^http/.test(uri.protocol)) {
      uri.searchParams.append("key", this.apiToken);
      if (this.sessionToken !== null) {
        uri.searchParams.append("session", this.sessionToken);
      }
    }
    return uri.toString();
  }
  dispose() {
    const { tiles } = this;
    tiles.removeEventListener("load-tile-set", this._onLoadCallback);
    tiles.removeEventListener("tile-visibility-change", this._visibilityChangeCallback);
  }
  async fetchData(uri, options) {
    if (this._tokenRefreshPromise !== null) {
      await this._tokenRefreshPromise;
      uri = this.preprocessURL(uri);
    }
    const res = await fetch(uri, options);
    if (res.status >= 400 && res.status <= 499 && this.autoRefreshToken) {
      await this._refreshToken(options);
      return fetch(this.preprocessURL(uri), options);
    } else {
      return res;
    }
  }
  _refreshToken(options) {
    if (this._tokenRefreshPromise === null) {
      const rootURL = new URL(this.tiles.rootURL);
      rootURL.searchParams.append("key", this.apiToken);
      this._tokenRefreshPromise = fetch(rootURL, options).then((res) => res.json()).then((res) => {
        this.sessionToken = getSessionToken(res.root);
        this._tokenRefreshPromise = null;
      });
    }
    return this._tokenRefreshPromise;
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/CesiumIonAuthPlugin.js
var CesiumIonAuthPlugin = class {
  constructor({ apiToken, assetId = null, autoRefreshToken = false }) {
    this.name = "CESIUM_ION_AUTH_PLUGIN";
    this.apiToken = apiToken;
    this.assetId = assetId;
    this.autoRefreshToken = autoRefreshToken;
    this.tiles = null;
    this.endpointURL = null;
    this._bearerToken = null;
    this._tileSetVersion = -1;
    this._tokenRefreshPromise = null;
    this._attributions = [];
  }
  init(tiles) {
    if (this.assetId !== null) {
      tiles.rootURL = `https://api.cesium.com/v1/assets/${this.assetId}/endpoint`;
    }
    this.tiles = tiles;
    this.endpointURL = tiles.rootURL;
    tiles.resetFailedTiles();
  }
  loadRootTileSet() {
    return this._refreshToken().then(() => this.tiles.loadRootTileSet());
  }
  preprocessURL(uri) {
    uri = new URL(uri);
    if (/^http/.test(uri.protocol) && this._tileSetVersion != -1) {
      uri.searchParams.append("v", this._tileSetVersion);
    }
    return uri.toString();
  }
  fetchData(uri, options) {
    const tiles = this.tiles;
    if (tiles.getPluginByName("GOOGLE_CLOUD_AUTH_PLUGIN") !== null) {
      return null;
    } else {
      return Promise.resolve().then(async () => {
        if (this._tokenRefreshPromise !== null) {
          await this._tokenRefreshPromise;
          uri = this.preprocessURL(uri);
        }
        const res = await fetch(uri, options);
        if (res.status >= 400 && res.status <= 499 && this.autoRefreshToken) {
          await this._refreshToken(options);
          return fetch(this.preprocessURL(uri), options);
        } else {
          return res;
        }
      });
    }
  }
  getAttributions(target) {
    if (this.tiles.visibleTiles.size > 0) {
      target.push(...this._attributions);
    }
  }
  _refreshToken(options) {
    if (this._tokenRefreshPromise === null) {
      const url = new URL(this.endpointURL);
      url.searchParams.append("access_token", this.apiToken);
      this._tokenRefreshPromise = fetch(url, options).then((res) => {
        if (!res.ok) {
          throw new Error(`CesiumIonAuthPlugin: Failed to load data with error code ${res.status}`);
        }
        return res.json();
      }).then((json) => {
        const tiles = this.tiles;
        if ("externalType" in json) {
          const url2 = new URL(json.options.url);
          tiles.rootURL = json.options.url;
          tiles.registerPlugin(new GoogleCloudAuthPlugin({ apiToken: url2.searchParams.get("key") }));
        } else {
          tiles.rootURL = json.url;
          tiles.fetchOptions.headers = tiles.fetchOptions.headers || {};
          tiles.fetchOptions.headers.Authorization = `Bearer ${json.accessToken}`;
          if (url.searchParams.has("v") && this._tileSetVersion === -1) {
            const url2 = new URL(json.url);
            this._tileSetVersion = url2.searchParams.get("v");
          }
          this._bearerToken = json.accessToken;
          if (json.attributions) {
            this._attributions = json.attributions.map((att) => ({
              value: att.html,
              type: "html",
              collapsible: att.collapsible
            }));
          }
        }
        this._tokenRefreshPromise = null;
        return json;
      });
    }
    return this._tokenRefreshPromise;
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/TileCompressionPlugin.js
import { Vector3 as Vector37, LinearFilter as LinearFilter2, BufferAttribute as BufferAttribute4, MathUtils as MathUtils4 } from "three";
var _vec4 = new Vector37();
function compressAttribute(attribute, arrayType) {
  if (attribute.isInterleavedBufferAttribute || attribute.array instanceof arrayType) {
    return attribute;
  }
  const signed = arrayType === Int8Array || arrayType === Int16Array || arrayType === Int32Array;
  const minValue = signed ? -1 : 0;
  const array = new arrayType(attribute.count * attribute.itemSize);
  const newAttribute = new BufferAttribute4(array, attribute.itemSize, true);
  const itemSize = attribute.itemSize;
  const count = attribute.count;
  for (let i = 0; i < count; i++) {
    for (let j = 0; j < itemSize; j++) {
      const v = MathUtils4.clamp(attribute.getComponent(i, j), minValue, 1);
      newAttribute.setComponent(i, j, v);
    }
  }
  return newAttribute;
}
function compressPositionAttribute(mesh, arrayType = Int16Array) {
  const geometry = mesh.geometry;
  const attributes = geometry.attributes;
  const attribute = attributes.position;
  if (attribute.isInterleavedBufferAttribute || attribute.array instanceof arrayType) {
    return attribute;
  }
  const array = new arrayType(attribute.count * attribute.itemSize);
  const newAttribute = new BufferAttribute4(array, attribute.itemSize, false);
  const itemSize = attribute.itemSize;
  const count = attribute.count;
  geometry.computeBoundingBox();
  const boundingBox = geometry.boundingBox;
  const { min, max } = boundingBox;
  const maxValue = 2 ** (8 * arrayType.BYTES_PER_ELEMENT - 1) - 1;
  const minValue = -maxValue;
  for (let i = 0; i < count; i++) {
    for (let j = 0; j < itemSize; j++) {
      const key = j === 0 ? "x" : j === 1 ? "y" : "z";
      const bbMinValue = min[key];
      const bbMaxValue = max[key];
      const v = MathUtils4.mapLinear(
        attribute.getComponent(i, j),
        bbMinValue,
        bbMaxValue,
        minValue,
        maxValue
      );
      newAttribute.setComponent(i, j, v);
    }
  }
  boundingBox.getCenter(_vec4);
  mesh.position.add(_vec4);
  mesh.scale.x *= 0.5 * (max.x - min.x) / maxValue;
  mesh.scale.y *= 0.5 * (max.y - min.y) / maxValue;
  mesh.scale.z *= 0.5 * (max.z - min.z) / maxValue;
  attributes.position = newAttribute;
  mesh.geometry.boundingBox = null;
  mesh.geometry.boundingSphere = null;
  mesh.updateMatrixWorld();
}
var TileCompressionPlugin = class {
  constructor(options) {
    this._options = {
      // whether to generate normals if they don't already exist.
      generateNormals: false,
      // whether to disable use of mipmaps since they are typically not necessary
      // with something like 3d tiles.
      disableMipmaps: true,
      // whether to compress certain attributes
      compressIndex: true,
      compressNormals: true,
      compressUvs: true,
      compressPosition: false,
      // the TypedArray type to use when compressing the attributes
      uvType: Int8Array,
      normalType: Int8Array,
      positionType: Int16Array,
      ...options
    };
  }
  processTileModel(scene, tile) {
    const {
      generateNormals,
      disableMipmaps,
      compressIndex,
      compressUvs,
      compressNormals,
      compressPosition,
      uvType,
      normalType,
      positionType
    } = this._options;
    scene.traverse((c) => {
      if (c.material && disableMipmaps) {
        const material = c.material;
        for (const key in material) {
          const value = material[key];
          if (value && value.isTexture && value.generateMipmaps) {
            value.generateMipmaps = false;
            value.minFilter = LinearFilter2;
          }
        }
      }
      if (c.geometry) {
        const geometry = c.geometry;
        const attributes = geometry.attributes;
        if (compressUvs) {
          const { uv, uv1, uv2, uv3 } = attributes;
          if (uv) attributes.uv = compressAttribute(uv, uvType);
          if (uv1) attributes.uv1 = compressAttribute(uv1, uvType);
          if (uv2) attributes.uv2 = compressAttribute(uv2, uvType);
          if (uv3) attributes.uv3 = compressAttribute(uv3, uvType);
        }
        if (generateNormals && !attributes.normals) {
          geometry.computeVertexNormals();
        }
        if (compressNormals && attributes.normals) {
          attributes.normals = compressAttribute(attributes.normals, normalType);
        }
        if (compressPosition) {
          compressPositionAttribute(c, positionType);
        }
        if (compressIndex && geometry.index) {
          const vertCount = attributes.position.count;
          const index = geometry.index;
          const type = vertCount > 65535 ? Uint32Array : vertCount > 255 ? Uint16Array : Uint8Array;
          if (!(index.array instanceof type)) {
            const array = new type(geometry.index.count);
            array.set(index.array);
            const attribute = new BufferAttribute4(array, 1);
            geometry.setIndex(attribute);
          }
        }
      }
    });
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/GLTFStructuralMetadataExtension.js
import { FileLoader as FileLoader2 } from "three";

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/utilities/ClassPropertyHelpers.js
import {
  Vector2 as Vector22,
  Vector3 as Vector38,
  Vector4,
  Matrix3,
  Matrix4 as Matrix46
} from "three";

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/math/Matrix2.js
var Matrix2 = class _Matrix2 {
  constructor(n11, n12, n21, n22) {
    _Matrix2.prototype.isMatrix2 = true;
    this.elements = [
      1,
      0,
      0,
      1
    ];
    if (n11 !== void 0) {
      this.set(n11, n12, n21, n22);
    }
  }
  identity() {
    this.set(
      1,
      0,
      0,
      1
    );
    return this;
  }
  fromArray(array, offset = 0) {
    for (let i = 0; i < 4; i++) {
      this.elements[i] = array[i + offset];
    }
    return this;
  }
  set(n11, n12, n21, n22) {
    const te = this.elements;
    te[0] = n11;
    te[1] = n21;
    te[2] = n12;
    te[3] = n22;
    return this;
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/utilities/ClassPropertyHelpers.js
function getField(object, key, def) {
  return object && key in object ? object[key] : def;
}
function isNumericType(type) {
  return type !== "BOOLEAN" && type !== "STRING" && type !== "ENUM";
}
function isFloatComponentType(type) {
  return /^FLOAT/.test(type);
}
function isVectorType(type) {
  return /^VEC/.test(type);
}
function isMatrixType(type) {
  return /^MATRIX/.test(type);
}
function readDataFromBufferToType(buffer, offset, type, target = null) {
  if (isMatrixType(type)) {
    return target.fromArray(buffer, offset);
  } else if (isVectorType(type)) {
    return target.fromArray(buffer, offset);
  } else {
    return buffer[offset];
  }
}
function getTypeInstance(property) {
  const { type, componentType } = property;
  switch (type) {
    case "SCALAR":
      return componentType === "INT64" ? 0n : 0;
    case "VEC2":
      return new Vector22();
    case "VEC3":
      return new Vector38();
    case "VEC4":
      return new Vector4();
    case "MAT2":
      return new Matrix2();
    case "MAT3":
      return new Matrix3();
    case "MAT4":
      return new Matrix46();
    case "BOOLEAN":
      return false;
    case "STRING":
      return "";
    // the final value for enums is a string but are represented as integers
    // during intermediate steps
    case "ENUM":
      return 0;
  }
}
function isTypeInstance(type, value) {
  if (value === null || value === void 0) {
    return false;
  }
  switch (type) {
    case "SCALAR":
      return typeof value === "number" || typeof value === "bigint";
    case "VEC2":
      return value.isVector2;
    case "VEC3":
      return value.isVector3;
    case "VEC4":
      return value.isVector4;
    case "MAT2":
      return value.isMatrix2;
    case "MAT3":
      return value.isMatrix3;
    case "MAT4":
      return value.isMatrix4;
    case "BOOLEAN":
      return typeof value === "boolean";
    case "STRING":
      return typeof value === "string";
    case "ENUM":
      return typeof value === "number" || typeof value === "bigint";
  }
  throw new Error("ClassProperty: invalid type.");
}
function getArrayConstructorFromComponentType(componentType, type = null) {
  switch (componentType) {
    case "INT8":
      return Int8Array;
    case "INT16":
      return Int16Array;
    case "INT32":
      return Int32Array;
    case "INT64":
      return BigInt64Array;
    case "UINT8":
      return Uint8Array;
    case "UINT16":
      return Uint16Array;
    case "UINT32":
      return Uint32Array;
    case "UINT64":
      return BigUint64Array;
    case "FLOAT32":
      return Float32Array;
    case "FLOAT64":
      return Float64Array;
  }
  switch (type) {
    case "BOOLEAN":
      return Uint8Array;
    case "STRING":
      return Uint8Array;
  }
  throw new Error("ClassProperty: invalid type.");
}
function resolveDefault(property, target = null) {
  const array = property.array;
  if (array) {
    target = target && Array.isArray(target) ? target : [];
    target.length = property.count;
    for (let i = 0, l = target.length; i < l; i++) {
      target[i] = resolveDefaultElement(property, target[i]);
    }
  } else {
    target = resolveDefaultElement(property, target);
  }
  return target;
}
function resolveDefaultElement(property, target = null) {
  const defaultValue = property.default;
  const type = property.type;
  target = target || getTypeInstance(property);
  if (defaultValue === null) {
    switch (type) {
      case "SCALAR":
        return 0;
      case "VEC2":
        return target.set(0, 0);
      case "VEC3":
        return target.set(0, 0, 0);
      case "VEC4":
        return target.set(0, 0, 0, 0);
      case "MAT2":
        return target.identity();
      case "MAT3":
        return target.identity();
      case "MAT4":
        return target.identity();
      case "BOOLEAN":
        return false;
      case "STRING":
        return "";
      case "ENUM":
        return "";
    }
    throw new Error("ClassProperty: invalid type.");
  } else {
    if (isMatrixType(type)) {
      target.fromArray(defaultValue);
    } else if (isVectorType(type)) {
      target.fromArray(defaultValue);
    } else {
      return defaultValue;
    }
  }
}
function resolveNoData(property, target) {
  if (property.noData === null) {
    return target;
  }
  const noData = property.noData;
  const type = property.type;
  if (Array.isArray(target)) {
    for (let i = 0, l = target.length; i < l; i++) {
      target[i] = performResolution(target[i]);
    }
  } else {
    target = performResolution(target);
  }
  return target;
  function performResolution(target2) {
    if (isNoDataEqual(target2)) {
      target2 = resolveDefaultElement(property, target2);
    }
    return target2;
  }
  function isNoDataEqual(value) {
    if (isMatrixType(type)) {
      const elements = value.elements;
      for (let i = 0, l = noData.length; i < l; i++) {
        if (noData[i] !== elements[i]) {
          return false;
        }
      }
      return true;
    } else if (isVectorType(type)) {
      for (let i = 0, l = noData.length; i < l; i++) {
        if (noData[i] !== value.getComponent(i)) {
          return false;
        }
      }
      return true;
    } else {
      return noData === value;
    }
  }
}
function normalizeValue(componentType, v) {
  switch (componentType) {
    case "INT8":
      return Math.max(v / 127, -1);
    case "INT16":
      return Math.max(v, 32767, -1);
    case "INT32":
      return Math.max(v / 2147483647, -1);
    case "INT64":
      return Math.max(Number(v) / 9223372036854776e3, -1);
    // eslint-disable-line no-loss-of-precision
    case "UINT8":
      return v / 255;
    case "UINT16":
      return v / 65535;
    case "UINT32":
      return v / 4294967295;
    case "UINT64":
      return Number(v) / 18446744073709552e3;
  }
}
function adjustValueScaleOffset(property, target) {
  const {
    type,
    componentType,
    scale,
    offset,
    normalized
  } = property;
  if (Array.isArray(target)) {
    for (let i = 0, l = target.length; i < l; i++) {
      target[i] = adjustFromType(target[i]);
    }
  } else {
    target = adjustFromType(target);
  }
  return target;
  function adjustFromType(value) {
    if (isMatrixType(type)) {
      value = adjustMatrix(value);
    } else if (isVectorType(type)) {
      value = adjustVector(value);
    } else {
      value = adjustScalar(value);
    }
    return value;
  }
  function adjustVector(value) {
    value.x = adjustScalar(value.x);
    value.y = adjustScalar(value.y);
    if ("z" in value) value.z = adjustScalar(value.z);
    if ("w" in value) value.w = adjustScalar(value.w);
    return value;
  }
  function adjustMatrix(value) {
    const elements = value.elements;
    for (let i = 0, l = elements.length; i < l; i++) {
      elements[i] = adjustScalar(elements[i]);
    }
    return value;
  }
  function adjustScalar(value) {
    if (normalized) {
      value = normalizeValue(componentType, value);
    }
    if (normalized || isFloatComponentType(componentType)) {
      value = value * scale + offset;
    }
    return value;
  }
}
function initializeFromProperty(property, target, overrideCount = null) {
  if (property.array) {
    if (!Array.isArray(target)) {
      target = new Array(property.count || 0);
    }
    target.length = overrideCount !== null ? overrideCount : property.count;
    for (let i = 0, l = target.length; i < l; i++) {
      if (!isTypeInstance(property.type, target[i])) {
        target[i] = getTypeInstance(property);
      }
    }
  } else {
    if (!isTypeInstance(property.type, target)) {
      target = getTypeInstance(property);
    }
  }
  return target;
}
function initializeFromClass(properties, target) {
  for (const key in target) {
    if (!(key in properties)) {
      delete target[key];
    }
  }
  for (const key in properties) {
    const prop = properties[key];
    target[key] = initializeFromProperty(prop, target[key]);
  }
}
function typeToComponentCount(type) {
  switch (type) {
    case "ENUM":
      return 1;
    case "SCALAR":
      return 1;
    case "VEC2":
      return 2;
    case "VEC3":
      return 3;
    case "VEC4":
      return 4;
    case "MAT2":
      return 4;
    case "MAT3":
      return 9;
    case "MAT4":
      return 16;
    // unused
    case "BOOLEAN":
      return -1;
    case "STRING":
      return -1;
    default:
      return -1;
  }
}

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/ClassProperty.js
var ClassProperty = class {
  constructor(enums, property, accessorProperty = null) {
    this.name = property.name || null;
    this.description = property.description || null;
    this.type = property.type;
    this.componentType = property.componentType || null;
    this.enumType = property.enumType || null;
    this.array = property.array || false;
    this.count = property.count || 0;
    this.normalized = property.normalized || false;
    this.offset = property.offset || 0;
    this.scale = getField(property, "scale", 1);
    this.max = getField(property, "max", Infinity);
    this.min = getField(property, "min", -Infinity);
    this.required = property.required || false;
    this.noData = getField(property, "noData", null);
    this.default = getField(property, "default", null);
    this.semantic = getField(property, "semantic", null);
    this.enumSet = null;
    this.accessorProperty = accessorProperty;
    if (accessorProperty) {
      this.offset = getField(accessorProperty, "offset", this.offset);
      this.scale = getField(accessorProperty, "scale", this.scale);
      this.max = getField(accessorProperty, "max", this.max);
      this.min = getField(accessorProperty, "min", this.min);
    }
    if (property.type === "ENUM") {
      this.enumSet = enums[this.enumType];
      if (this.componentType === null) {
        this.componentType = getField(this.enumSet, "valueType", "UINT16");
      }
    }
  }
  // shape the given target to match the data type of the property
  // enums are set to their integer value
  shapeToProperty(target, countOverride = null) {
    return initializeFromProperty(this, target, countOverride);
  }
  // resolve the given object to the default value for the property for a single element
  // enums are set to a default string
  resolveDefaultElement(target) {
    return resolveDefaultElement(this, target);
  }
  // resolve the target to the default value for the property for every element if it's an array
  // enums are set to a default string
  resolveDefault(target) {
    return resolveDefault(this, target);
  }
  // converts any instances of no data to the default value
  resolveNoData(target) {
    return resolveNoData(this, target);
  }
  // converts enums integers in the given target to strings
  resolveEnumsToStrings(target) {
    const enumSet = this.enumSet;
    if (this.type === "ENUM") {
      if (Array.isArray(target)) {
        for (let i = 0, l = target.length; i < l; i++) {
          target[i] = getEnumName(target[i]);
        }
      } else {
        target = getEnumName(target);
      }
    }
    return target;
    function getEnumName(index) {
      const match = enumSet.values.find((e) => e.value === index);
      if (match === null) {
        return "";
      } else {
        return match.name;
      }
    }
  }
  // apply scales
  adjustValueScaleOffset(target) {
    if (isNumericType(this.type)) {
      return adjustValueScaleOffset(this, target);
    } else {
      return target;
    }
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/PropertySetAccessor.js
var PropertySetAccessor = class {
  constructor(definition, classes = {}, enums = {}, data = null) {
    this.definition = definition;
    this.class = classes[definition.class];
    this.className = definition.class;
    this.enums = enums;
    this.data = data;
    this.name = "name" in definition ? definition.name : null;
    this.properties = null;
  }
  getPropertyNames() {
    return Object.keys(this.class.properties);
  }
  includesData(name2) {
    return Boolean(this.definition.properties[name2]);
  }
  dispose() {
  }
  _initProperties(propertyClass = ClassProperty) {
    const properties = {};
    for (const key in this.class.properties) {
      properties[key] = new propertyClass(this.enums, this.class.properties[key], this.definition.properties[key]);
    }
    this.properties = properties;
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/PropertyAttributeAccessor.js
var PropertyAttributeClassProperty = class extends ClassProperty {
  constructor(enums, classProperty, attributeProperty = null) {
    super(enums, classProperty, attributeProperty);
    this.attribute = attributeProperty.attribute;
  }
};
var PropertyAttributeAccessor = class extends PropertySetAccessor {
  constructor(...args) {
    super(...args);
    this.isPropertyAttributeAccessor = true;
    this._initProperties(PropertyAttributeClassProperty);
  }
  getData(id, geometry, target = {}) {
    const properties = this.properties;
    initializeFromClass(properties, target);
    for (const name2 in properties) {
      target[name2] = this.getPropertyValue(name2, id, geometry, target[name2]);
    }
    return target;
  }
  getPropertyValue(name2, id, geometry, target = null) {
    if (id >= this.count) {
      throw new Error("PropertyAttributeAccessor: Requested index is outside the range of the buffer.");
    }
    const property = this.properties[name2];
    const type = property.type;
    if (!property) {
      throw new Error("PropertyAttributeAccessor: Requested class property does not exist.");
    } else if (!this.definition.properties[name2]) {
      return property.resolveDefault(target);
    }
    target = property.shapeToProperty(target);
    const attribute = geometry.getAttribute(property.attribute.toLowerCase());
    if (isMatrixType(type)) {
      const elements = target.elements;
      for (let i = 0, l = elements.length; i < l; i < l) {
        elements[i] = attribute.getComponent(id, i);
      }
    } else if (isVectorType(type)) {
      target.fromBufferAttribute(attribute, id);
    } else if (type === "SCALAR" || type === "ENUM") {
      target = attribute.getX(id);
    } else {
      throw new Error("StructuredMetadata.PropertyAttributeAccessor: BOOLEAN and STRING types are not supported by property attributes.");
    }
    target = property.adjustValueScaleOffset(target);
    target = property.resolveEnumsToStrings(target);
    target = property.resolveNoData(target);
    return target;
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/PropertyTableAccessor.js
var PropertyTableClassProperty = class extends ClassProperty {
  constructor(enums, classProperty, tableProperty = null) {
    super(enums, classProperty, tableProperty);
    this.values = tableProperty.values;
    this.valueLength = typeToComponentCount(this.type);
    this.arrayOffsets = getField(tableProperty, "arrayOffsets", null);
    this.stringOffsets = getField(tableProperty, "stringOffsets", null);
    this.arrayOffsetType = getField(tableProperty, "arrayOffsetType", "UINT32");
    this.stringOffsetType = getField(tableProperty, "stringOffsetType", "UINT32");
  }
  // returns the necessary array length based on the array offsets if present
  getArrayLengthFromId(buffers, id) {
    let count = this.count;
    if (this.arrayOffsets !== null) {
      const { arrayOffsets, arrayOffsetType } = this;
      const bufferCons = getArrayConstructorFromComponentType(arrayOffsetType);
      const arr = new bufferCons(buffers[arrayOffsets]);
      count = arr[id + 1] - arr[id];
    }
    return count;
  }
  // returns the index offset into the data buffer for the given id based on the
  // the array offsets if present
  getIndexOffsetFromId(buffers, id) {
    let indexOffset = id;
    if (this.arrayOffsets) {
      const { arrayOffsets, arrayOffsetType } = this;
      const bufferCons = getArrayConstructorFromComponentType(arrayOffsetType);
      const arr = new bufferCons(buffers[arrayOffsets]);
      indexOffset = arr[indexOffset];
    } else if (this.array) {
      indexOffset *= this.count;
    }
    return indexOffset;
  }
};
var PropertyTableAccessor = class extends PropertySetAccessor {
  constructor(...args) {
    super(...args);
    this.isPropertyTableAccessor = true;
    this.count = this.definition.count;
    this._initProperties(PropertyTableClassProperty);
  }
  getData(id, target = {}) {
    const properties = this.properties;
    initializeFromClass(properties, target);
    for (const name2 in properties) {
      target[name2] = this.getPropertyValue(name2, id, target[name2]);
    }
    return target;
  }
  // reads an individual element
  _readValueAtIndex(name2, id, index, target = null) {
    const property = this.properties[name2];
    const { componentType, type } = property;
    const buffers = this.data;
    const bufferView = buffers[property.values];
    const bufferCons = getArrayConstructorFromComponentType(componentType, type);
    const dataArray = new bufferCons(bufferView);
    const indexOffset = property.getIndexOffsetFromId(buffers, id);
    if (isNumericType(type) || type === "ENUM") {
      return readDataFromBufferToType(dataArray, (indexOffset + index) * property.valueLength, type, target);
    } else if (type === "STRING") {
      let stringIndex = indexOffset + index;
      let stringLength = 0;
      if (property.stringOffsets !== null) {
        const { stringOffsets, stringOffsetType } = property;
        const bufferCons2 = getArrayConstructorFromComponentType(stringOffsetType);
        const stringOffsetBuffer = new bufferCons2(buffers[stringOffsets]);
        stringLength = stringOffsetBuffer[stringIndex + 1] - stringOffsetBuffer[stringIndex];
        stringIndex = stringOffsetBuffer[stringIndex];
      }
      const byteArray = new Uint8Array(dataArray.buffer, stringIndex, stringLength);
      target = new TextDecoder().decode(byteArray);
    } else if (type === "BOOLEAN") {
      const offset = indexOffset + index;
      const byteIndex = Math.floor(offset / 8);
      const bitIndex = offset % 8;
      const bitValue = dataArray[byteIndex] >> bitIndex & 1;
      target = bitValue === 1;
    }
    return target;
  }
  // Reads the data for the given table index
  getPropertyValue(name2, id, target = null) {
    if (id >= this.count) {
      throw new Error("PropertyTableAccessor: Requested index is outside the range of the table.");
    }
    const property = this.properties[name2];
    if (!property) {
      throw new Error("PropertyTableAccessor: Requested property does not exist.");
    } else if (!this.definition.properties[name2]) {
      return property.resolveDefault(target);
    }
    const array = property.array;
    const buffers = this.data;
    const count = property.getArrayLengthFromId(buffers, id);
    target = property.shapeToProperty(target, count);
    if (array) {
      for (let i = 0, l = target.length; i < l; i++) {
        target[i] = this._readValueAtIndex(name2, id, i, target[i]);
      }
    } else {
      target = this._readValueAtIndex(name2, id, 0, target);
    }
    target = property.adjustValueScaleOffset(target);
    target = property.resolveEnumsToStrings(target);
    target = property.resolveNoData(target);
    return target;
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/PropertyTextureAccessor.js
import { Vector2 as Vector25 } from "three";

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/utilities/TextureReadUtility.js
import { WebGLRenderTarget, WebGLRenderer, Box2, Vector2 as Vector23, Vector4 as Vector42, ShaderMaterial, REVISION, CustomBlending, ZeroFactor, OneFactor } from "three";

// node_modules/three/examples/jsm/postprocessing/Pass.js
import {
  BufferGeometry as BufferGeometry4,
  Float32BufferAttribute as Float32BufferAttribute2,
  OrthographicCamera as OrthographicCamera2,
  Mesh as Mesh2
} from "three";
var _camera = new OrthographicCamera2(-1, 1, 1, -1, 0, 1);
var FullscreenTriangleGeometry = class extends BufferGeometry4 {
  constructor() {
    super();
    this.setAttribute("position", new Float32BufferAttribute2([-1, 3, 0, -1, -1, 0, 3, -1, 0], 3));
    this.setAttribute("uv", new Float32BufferAttribute2([0, 2, 0, 0, 2, 0], 2));
  }
};
var _geometry = new FullscreenTriangleGeometry();
var FullScreenQuad = class {
  constructor(material) {
    this._mesh = new Mesh2(_geometry, material);
  }
  dispose() {
    this._mesh.geometry.dispose();
  }
  render(renderer) {
    renderer.render(this._mesh, _camera);
  }
  get material() {
    return this._mesh.material;
  }
  set material(value) {
    this._mesh.material = value;
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/utilities/TextureReadUtility.js
var REVISION_GEQ_165 = parseInt(REVISION) >= 165;
var REVISION_GEQ_166 = parseInt(REVISION) >= 166;
var _box = /* @__PURE__ */ new Box2();
var _currentScissor = /* @__PURE__ */ new Vector42();
var _pos2 = /* @__PURE__ */ new Vector23();
var TextureReadUtility = new class {
  constructor() {
    this._renderer = new WebGLRenderer();
    this._target = new WebGLRenderTarget(1, 1);
    this._texTarget = new WebGLRenderTarget();
    this._quad = new FullScreenQuad(new ShaderMaterial({
      blending: CustomBlending,
      blendDst: ZeroFactor,
      blendSrc: OneFactor,
      uniforms: {
        map: { value: null },
        pixel: { value: new Vector23() }
      },
      vertexShader: (
        /* glsl */
        `
				void main() {

					gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );

				}
			`
      ),
      fragmentShader: (
        /* glsl */
        `
				uniform sampler2D map;
				uniform ivec2 pixel;

				void main() {

					gl_FragColor = texelFetch( map, pixel, 0 );

				}
			`
      )
    }));
  }
  // increases the width of the target render target to support more data
  increaseSizeTo(width) {
    this._target.setSize(Math.max(this._target.width, width), 1);
  }
  // read data from the rendered texture asynchronously
  readDataAsync(buffer) {
    const { _renderer, _target } = this;
    if (REVISION_GEQ_165) {
      return _renderer.readRenderTargetPixelsAsync(_target, 0, 0, buffer.length / 4, 1, buffer);
    } else {
      return Promise.resolve().then(() => this.readData(buffer));
    }
  }
  // read data from the rendered texture
  readData(buffer) {
    const { _renderer, _target } = this;
    _renderer.readRenderTargetPixels(_target, 0, 0, buffer.length / 4, 1, buffer);
  }
  // render a single pixel from the source at the destination point on the render target
  // takes the texture, pixel to read from, and pixel to render in to
  renderPixelToTarget(texture, pixel, dstPixel) {
    const { _quad, _renderer, _target, _texTarget } = this;
    if (REVISION_GEQ_166) {
      _box.min.copy(pixel);
      _box.max.copy(pixel);
      _box.max.x += 1;
      _box.max.y += 1;
      _renderer.initRenderTarget(_target);
      _renderer.copyTextureToTexture(texture, _target.texture, _box, dstPixel, 0);
    } else {
      const currentAutoClear = _renderer.autoClear;
      const currentTarget = _renderer.getRenderTarget();
      const currentScissorTest = _renderer.getScissorTest();
      _renderer.getScissor(_currentScissor);
      _texTarget.setSize(texture.image.width, texture.image.height);
      _renderer.setRenderTarget(_texTarget);
      _pos2.set(0, 0);
      if (REVISION_GEQ_165) {
        _renderer.copyTextureToTexture(texture, _texTarget.texture, null, _pos2);
      } else {
        _renderer.copyTextureToTexture(_pos2, texture, _texTarget.texture);
      }
      _quad.material.uniforms.map.value = _texTarget.texture;
      _quad.material.uniforms.pixel.value.copy(pixel);
      _renderer.setRenderTarget(_target);
      _renderer.setScissorTest(true);
      _renderer.setScissor(dstPixel.x, dstPixel.y, 1, 1);
      _renderer.autoClear = false;
      _quad.render(_renderer);
      _renderer.setScissorTest(currentScissorTest);
      _renderer.setScissor(_currentScissor);
      _renderer.setRenderTarget(currentTarget);
      _renderer.autoClear = currentAutoClear;
      _texTarget.dispose();
    }
  }
}();

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/utilities/TexCoordUtilities.js
import { Vector2 as Vector24 } from "three";
var _uv0 = /* @__PURE__ */ new Vector24();
var _uv1 = /* @__PURE__ */ new Vector24();
var _uv2 = /* @__PURE__ */ new Vector24();
function getTextureCoordAttribute(geometry, index) {
  if (index === 0) {
    return geometry.getAttribute("uv");
  } else {
    return geometry.getAttribute(`uv${index}`);
  }
}
function getTriangleVertexIndices(geometry, faceIndex, target = new Array(3)) {
  let i0 = 3 * faceIndex;
  let i1 = 3 * faceIndex + 1;
  let i2 = 3 * faceIndex + 2;
  if (geometry.index) {
    i0 = geometry.index.getX(i0);
    i1 = geometry.index.getX(i1);
    i2 = geometry.index.getX(i2);
  }
  target[0] = i0;
  target[1] = i1;
  target[2] = i2;
  return target;
}
function getTexCoord(geometry, texCoord, barycoord, indices, target) {
  const [i0, i1, i2] = indices;
  const attr = getTextureCoordAttribute(geometry, texCoord);
  _uv0.fromBufferAttribute(attr, i0);
  _uv1.fromBufferAttribute(attr, i1);
  _uv2.fromBufferAttribute(attr, i2);
  target.set(0, 0, 0).addScaledVector(_uv0, barycoord.x).addScaledVector(_uv1, barycoord.y).addScaledVector(_uv2, barycoord.z);
}
function getTexelIndices(uv, width, height, target) {
  const fx = uv.x - Math.floor(uv.x);
  const fy = uv.y - Math.floor(uv.y);
  const px = Math.floor(fx * width % width);
  const py = Math.floor(fy * height % height);
  target.set(px, py);
  return target;
}

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/PropertyTextureAccessor.js
var _uv = /* @__PURE__ */ new Vector25();
var _srcPixel = /* @__PURE__ */ new Vector25();
var _dstPixel = /* @__PURE__ */ new Vector25();
var PropertyTextureClassProperty = class extends ClassProperty {
  constructor(enums, classProperty, textureProperty = null) {
    super(enums, classProperty, textureProperty);
    this.channels = getField(textureProperty, "channels", [0]);
    this.index = getField(textureProperty, "index", null);
    this.texCoord = getField(textureProperty, "texCoord", null);
    this.valueLength = parseInt(this.type.replace(/[^0-9]/g, "")) || 1;
  }
  // takes the buffer to read from and the value index to read
  readDataFromBuffer(buffer, index, target = null) {
    const type = this.type;
    if (type === "BOOLEAN" || type === "STRING") {
      throw new Error("PropertyTextureAccessor: BOOLEAN and STRING types not supported.");
    }
    return readDataFromBufferToType(buffer, index * this.valueLength, type, target);
  }
};
var PropertyTextureAccessor = class extends PropertySetAccessor {
  constructor(...args) {
    super(...args);
    this.isPropertyTextureAccessor = true;
    this._asyncRead = false;
    this._initProperties(PropertyTextureClassProperty);
  }
  // Reads the full set of property data
  getData(faceIndex, barycoord, geometry, target = {}) {
    const properties = this.properties;
    initializeFromClass(properties, target);
    const names = Object.keys(properties);
    const results = names.map((n) => target[n]);
    this.getPropertyValuesAtTexel(names, faceIndex, barycoord, geometry, results);
    names.forEach((n, i) => target[n] = results[i]);
    return target;
  }
  // Reads the full set of property data asynchronously
  async getDataAsync(faceIndex, barycoord, geometry, target = {}) {
    const properties = this.properties;
    initializeFromClass(properties, target);
    const names = Object.keys(properties);
    const results = names.map((n) => target[n]);
    await this.getPropertyValuesAtTexelAsync(names, faceIndex, barycoord, geometry, results);
    names.forEach((n, i) => target[n] = results[i]);
    return target;
  }
  // Reads values asynchronously
  getPropertyValuesAtTexelAsync(...args) {
    this._asyncRead = true;
    const result = this.getPropertyValuesAtTexel(...args);
    this._asyncRead = false;
    return result;
  }
  // Reads values from the textures synchronously
  getPropertyValuesAtTexel(names, faceIndex, barycoord, geometry, target = []) {
    while (target.length < names.length) target.push(null);
    target.length = names.length;
    TextureReadUtility.increaseSizeTo(target.length);
    const textures = this.data;
    const accessorProperties = this.definition.properties;
    const properties = this.properties;
    const indices = getTriangleVertexIndices(geometry, faceIndex);
    for (let i = 0, l = names.length; i < l; i++) {
      const name2 = names[i];
      if (!accessorProperties[name2]) {
        continue;
      }
      const property = properties[name2];
      const texture = textures[property.index];
      getTexCoord(geometry, property.texCoord, barycoord, indices, _uv);
      getTexelIndices(_uv, texture.image.width, texture.image.height, _srcPixel);
      _dstPixel.set(i, 0);
      TextureReadUtility.renderPixelToTarget(texture, _srcPixel, _dstPixel);
    }
    const buffer = new Uint8Array(names.length * 4);
    if (this._asyncRead) {
      return TextureReadUtility.readDataAsync(buffer).then(() => {
        readTextureSampleResults.call(this);
        return target;
      });
    } else {
      TextureReadUtility.readData(buffer);
      readTextureSampleResults.call(this);
      return target;
    }
    function readTextureSampleResults() {
      for (let i = 0, l = names.length; i < l; i++) {
        const name2 = names[i];
        const property = properties[name2];
        const type = property.type;
        target[i] = initializeFromProperty(property, target[i]);
        if (!property) {
          throw new Error("PropertyTextureAccessor: Requested property does not exist.");
        } else if (!accessorProperties[name2]) {
          target[i] = property.resolveDefault(target);
          continue;
        }
        const length = property.valueLength * (property.count || 1);
        const data = property.channels.map((c) => buffer[4 * i + c]);
        const componentType = property.componentType;
        const BufferCons = getArrayConstructorFromComponentType(componentType, type);
        const readBuffer = new BufferCons(length);
        new Uint8Array(readBuffer.buffer).set(data);
        if (property.array) {
          const arr = target[i];
          for (let j = 0, lj = arr.length; j < lj; j++) {
            arr[j] = property.readDataFromBuffer(readBuffer, j, arr[j]);
          }
        } else {
          target[i] = property.readDataFromBuffer(readBuffer, 0, target[i]);
        }
        target[i] = property.adjustValueScaleOffset(target[i]);
        target[i] = property.resolveEnumsToStrings(target[i]);
        target[i] = property.resolveNoData(target[i]);
      }
    }
  }
  // dispose all of the texture data used
  dispose() {
    this.data.forEach((texture) => {
      if (texture) {
        texture.dispose();
        if (texture.image instanceof ImageBitmap) {
          texture.image.close();
        }
      }
    });
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/StructuralMetadata.js
var StructuralMetadata = class {
  constructor(definition, textures, buffers, nodeMetadata = null, object = null) {
    const {
      schema,
      propertyTables = [],
      propertyTextures = [],
      propertyAttributes = []
    } = definition;
    const { enums, classes } = schema;
    const tableAccessors = propertyTables.map((t) => new PropertyTableAccessor(t, classes, enums, buffers));
    let textureAccessors = [];
    let attributeAccessors = [];
    if (nodeMetadata) {
      if (nodeMetadata.propertyTextures) {
        textureAccessors = nodeMetadata.propertyTextures.map((i) => new PropertyTextureAccessor(propertyTextures[i], classes, enums, textures));
      }
      if (nodeMetadata.propertyAttributes) {
        attributeAccessors = nodeMetadata.propertyAttributes.map((i) => new PropertyAttributeAccessor(propertyAttributes[i], classes, enums));
      }
    }
    this.schema = schema;
    this.tableAccessors = tableAccessors;
    this.textureAccessors = textureAccessors;
    this.attributeAccessors = attributeAccessors;
    this.object = object;
    this.textures = textures;
    this.nodeMetadata = nodeMetadata;
  }
  // Property Tables
  getPropertyTableData(tableIndices, ids, target = null) {
    if (!Array.isArray(tableIndices) || !Array.isArray(ids)) {
      target = target || {};
      const table = this.tableAccessors[tableIndices];
      target = table.getData(ids, target);
    } else {
      target = target || [];
      const length = Math.min(tableIndices.length, ids.length);
      target.length = length;
      for (let i = 0; i < length; i++) {
        const table = this.tableAccessors[tableIndices[i]];
        target[i] = table.getData(ids[i], target[i]);
      }
    }
    return target;
  }
  getPropertyTableInfo(tableIndices = null) {
    if (tableIndices === null) {
      tableIndices = this.tableAccessors.map((t, i) => i);
    }
    if (Array.isArray(tableIndices)) {
      return tableIndices.map((i) => {
        const table = this.tableAccessors[i];
        return {
          name: table.name,
          className: table.definition.class
        };
      });
    } else {
      const table = this.tableAccessors[tableIndices];
      return {
        name: table.name,
        className: table.definition.class
      };
    }
  }
  // Property Textures
  getPropertyTextureData(triangle, barycoord, target = []) {
    const textureAccessors = this.textureAccessors;
    target.length = textureAccessors.length;
    for (let i = 0; i < textureAccessors.length; i++) {
      const accessor = textureAccessors[i];
      target[i] = accessor.getData(triangle, barycoord, this.object.geometry, target[i]);
    }
    return target;
  }
  async getPropertyTextureDataAsync(triangle, barycoord, target = []) {
    const textureAccessors = this.textureAccessors;
    target.length = textureAccessors.length;
    const promises = [];
    for (let i = 0; i < textureAccessors.length; i++) {
      const accessor = textureAccessors[i];
      const promise = accessor.getDataAsync(triangle, barycoord, this.object.geometry, target[i]).then((result) => {
        target[i] = result;
      });
      promises.push(promise);
    }
    await Promise.all(promises);
    return target;
  }
  getPropertyTextureInfo() {
    return this.textureAccessors;
  }
  // Property Attributes
  getPropertyAttributeData(attributeIndex, target = []) {
    const attributeAccessors = this.attributeAccessors;
    target.length = attributeAccessors.length;
    for (let i = 0; i < attributeAccessors.length; i++) {
      const accessor = attributeAccessors[i];
      target[i] = accessor.getData(attributeIndex, this.object.geometry, target[i]);
    }
    return target;
  }
  getPropertyAttributeInfo() {
    return this.attributeAccessors.map((acc) => {
      return {
        name: acc.name,
        className: acc.definition.class
      };
    });
  }
  dispose() {
    this.textureAccessors.forEach((acc) => acc.dispose());
    this.tableAccessors.forEach((acc) => acc.dispose());
    this.attributeAccessors.forEach((acc) => acc.dispose());
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/GLTFStructuralMetadataExtension.js
var EXT_NAME = "EXT_structural_metadata";
function getRelevantTextures(parser, propertyTextures = []) {
  const textureCount = parser.json.textures?.length || 0;
  const result = new Array(textureCount).fill(null);
  propertyTextures.forEach(({ properties }) => {
    for (const key in properties) {
      const { index } = properties[key];
      if (result[index] === null) {
        result[index] = parser.loadTexture(index);
      }
    }
  });
  return Promise.all(result);
}
function getRelevantBuffers(parser, propertyTables = []) {
  const textureCount = parser.json.bufferViews?.length || 0;
  const result = new Array(textureCount).fill(null);
  propertyTables.forEach(({ properties }) => {
    for (const key in properties) {
      const { values, arrayOffsets, stringOffsets } = properties[key];
      if (result[values] === null) {
        result[values] = parser.loadBufferView(values);
      }
      if (result[arrayOffsets] === null) {
        result[arrayOffsets] = parser.loadBufferView(arrayOffsets);
      }
      if (result[stringOffsets] === null) {
        result[stringOffsets] = parser.loadBufferView(stringOffsets);
      }
    }
  });
  return Promise.all(result);
}
var GLTFStructuralMetadataExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXT_NAME;
  }
  async afterRoot({ scene, parser }) {
    const extensionsUsed = parser.json.extensionsUsed;
    if (!extensionsUsed || !extensionsUsed.includes(EXT_NAME)) {
      return;
    }
    let schemaPromise = null;
    let rootExtension = parser.json.extensions[EXT_NAME];
    if (rootExtension.schemaUri) {
      const { manager, path, requestHeader, crossOrigin } = parser.options;
      const finalUri = new URL(rootExtension.schemaUri, path).toString();
      const fileLoader = new FileLoader2(manager);
      fileLoader.setCrossOrigin(crossOrigin);
      fileLoader.setResponseType("json");
      fileLoader.setRequestHeader(requestHeader);
      schemaPromise = fileLoader.loadAsync(finalUri).then((schema) => {
        rootExtension = { ...rootExtension, schema };
      });
    }
    const [textures, buffers] = await Promise.all([
      getRelevantTextures(parser, rootExtension.propertyTextures),
      getRelevantBuffers(parser, rootExtension.propertyTables),
      schemaPromise
    ]);
    const rootMetadata = new StructuralMetadata(rootExtension, textures, buffers);
    scene.userData.structuralMetadata = rootMetadata;
    scene.traverse((child) => {
      if (parser.associations.has(child)) {
        const { meshes, primitives } = parser.associations.get(child);
        const primitive = parser.json.meshes[meshes].primitives[primitives];
        if (primitive && primitive.extensions && primitive.extensions[EXT_NAME]) {
          const extension = primitive.extensions[EXT_NAME];
          child.userData.structuralMetadata = new StructuralMetadata(rootExtension, textures, buffers, extension, child);
        } else {
          child.userData.structuralMetadata = rootMetadata;
        }
      }
    });
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/metadata/classes/MeshFeatures.js
import { Vector2 as Vector26 } from "three";
var _uv3 = /* @__PURE__ */ new Vector26();
var _pixel = /* @__PURE__ */ new Vector26();
var _dstPixel2 = /* @__PURE__ */ new Vector26();
function getMaxBarycoordIndex(barycoord) {
  if (barycoord.x > barycoord.y && barycoord.x > barycoord.z) {
    return 0;
  } else if (barycoord.y > barycoord.z) {
    return 1;
  } else {
    return 2;
  }
}
var MeshFeatures = class {
  constructor(geometry, textures, data) {
    this.geometry = geometry;
    this.textures = textures;
    this.data = data;
    this._asyncRead = false;
    this.featureIds = data.featureIds.map((info) => {
      const { texture, ...rest } = info;
      const result = {
        label: null,
        propertyTable: null,
        nullFeatureId: null,
        ...rest
      };
      if (texture) {
        result.texture = {
          texCoord: 0,
          channels: [0],
          ...texture
        };
      }
      return result;
    });
  }
  // returns list of textures
  getTextures() {
    return this.textures;
  }
  // returns a set of info for each feature
  getFeatureInfo() {
    return this.featureIds;
  }
  // performs texture data read back asynchronously
  getFeaturesAsync(...args) {
    this._asyncRead = true;
    const result = this.getFeatures(...args);
    this._asyncRead = false;
    return result;
  }
  // returns all features for the given point on the given triangle
  getFeatures(triangle, barycoord) {
    const { geometry, textures, featureIds } = this;
    const result = new Array(featureIds.length).fill(null);
    const width = featureIds.length;
    TextureReadUtility.increaseSizeTo(width);
    const indices = getTriangleVertexIndices(geometry, triangle);
    const closestIndex = indices[getMaxBarycoordIndex(barycoord)];
    for (let i = 0, l = featureIds.length; i < l; i++) {
      const featureId = featureIds[i];
      const nullFeatureId = "nullFeatureId" in featureId ? featureId.nullFeatureId : null;
      if ("texture" in featureId) {
        const texture = textures[featureId.texture.index];
        getTexCoord(geometry, featureId.texture.texCoord, barycoord, indices, _uv3);
        getTexelIndices(_uv3, texture.image.width, texture.image.height, _pixel);
        _dstPixel2.set(i, 0);
        TextureReadUtility.renderPixelToTarget(textures[featureId.texture.index], _pixel, _dstPixel2);
      } else if ("attribute" in featureId) {
        const attr = geometry.getAttribute(`_feature_id_${featureId.attribute}`);
        const value = attr.getX(closestIndex);
        if (value !== nullFeatureId) {
          result[i] = value;
        }
      } else {
        const value = closestIndex;
        if (value !== nullFeatureId) {
          result[i] = value;
        }
      }
    }
    const buffer = new Uint8Array(width * 4);
    if (this._asyncRead) {
      return TextureReadUtility.readDataAsync(buffer).then(() => {
        readTextureSampleResults();
        return result;
      });
    } else {
      TextureReadUtility.readData(buffer);
      readTextureSampleResults();
      return result;
    }
    function readTextureSampleResults() {
      const readBuffer = new Uint32Array(1);
      for (let i = 0, l = featureIds.length; i < l; i++) {
        const featureId = featureIds[i];
        const nullFeatureId = "nullFeatureId" in featureId ? featureId.nullFeatureId : null;
        if ("texture" in featureId) {
          const { channels } = featureId.texture;
          const data = channels.map((c) => buffer[4 * i + c]);
          new Uint8Array(readBuffer.buffer).set(data);
          const value = readBuffer[0];
          if (value !== nullFeatureId) {
            result[i] = value;
          }
        }
      }
    }
  }
  // dispose all of the texture data used
  dispose() {
    this.textures.forEach((texture) => {
      if (texture) {
        texture.dispose();
        if (texture.image instanceof ImageBitmap) {
          texture.image.close();
        }
      }
    });
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/GLTFMeshFeaturesExtension.js
var EXT_NAME2 = "EXT_mesh_features";
function forEachPrimitiveExtension(scene, parser, callback) {
  scene.traverse((c) => {
    if (parser.associations.has(c)) {
      const { meshes, primitives } = parser.associations.get(c);
      const primitive = parser.json.meshes[meshes].primitives[primitives];
      if (primitive && primitive.extensions && primitive.extensions[EXT_NAME2]) {
        callback(c, primitive.extensions[EXT_NAME2]);
      }
    }
  });
}
var GLTFMeshFeaturesExtension = class {
  constructor(parser) {
    this.parser = parser;
    this.name = EXT_NAME2;
  }
  async afterRoot({ scene, parser }) {
    const extensionsUsed = parser.json.extensionsUsed;
    if (!extensionsUsed || !extensionsUsed.includes(EXT_NAME2)) {
      return;
    }
    const textureCount = parser.json.textures?.length || 0;
    const promises = new Array(textureCount).fill(null);
    forEachPrimitiveExtension(scene, parser, (child, { featureIds }) => {
      featureIds.forEach((info) => {
        if (info.texture && promises[info.texture.index] === null) {
          const index = info.texture.index;
          promises[index] = parser.loadTexture(index);
        }
      });
    });
    const textures = await Promise.all(promises);
    forEachPrimitiveExtension(scene, parser, (child, extension) => {
      child.userData.meshFeatures = new MeshFeatures(child.geometry, textures, extension);
    });
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/gltf/GLTFCesiumRTCExtension.js
var GLTFCesiumRTCExtension = class {
  constructor() {
    this.name = "CESIUM_RTC";
  }
  afterRoot(res) {
    if (res.parser.json.extensions && res.parser.json.extensions.CESIUM_RTC) {
      const { center } = res.parser.json.extensions.CESIUM_RTC;
      if (center) {
        res.scene.position.x += center[0];
        res.scene.position.y += center[1];
        res.scene.position.z += center[2];
      }
    }
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/GLTFExtensionsPlugin.js
var GLTFExtensionsPlugin = class {
  constructor(options) {
    options = {
      metadata: true,
      rtc: true,
      plugins: [],
      dracoLoader: null,
      ktxLoader: null,
      autoDispose: true,
      ...options
    };
    this.tiles = null;
    this.metadata = options.metadata;
    this.rtc = options.rtc;
    this.plugins = options.plugins;
    this.dracoLoader = options.dracoLoader;
    this.ktxLoader = options.ktxLoader;
    this._gltfRegex = /\.(gltf|glb)$/g;
    this._dracoRegex = /\.drc$/g;
    this._loader = null;
  }
  init(tiles) {
    const loader = new GLTFLoader(tiles.manager);
    if (this.dracoLoader) {
      loader.setDRACOLoader(this.dracoLoader);
      tiles.manager.addHandler(this._dracoRegex, this.dracoLoader);
    }
    if (this.ktxLoader) {
      loader.setKTX2Loader(this.ktxLoader);
    }
    if (this.rtc) {
      loader.register(() => new GLTFCesiumRTCExtension());
    }
    if (this.metadata) {
      loader.register(() => new GLTFStructuralMetadataExtension());
      loader.register(() => new GLTFMeshFeaturesExtension());
    }
    this.plugins.forEach((plugin) => loader.register(plugin));
    tiles.manager.addHandler(this._gltfRegex, loader);
    this.tiles = tiles;
    this._loader = loader;
  }
  dispose() {
    this.tiles.manager.removeHandler(this._gltfRegex);
    this.tiles.manager.removeHandler(this._dracoRegex);
    if (this.autoDispose) {
      this.ktxLoader.dispose();
      this.dracoLoader.dispose();
    }
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/fade/TilesFadePlugin.js
import { Group as Group3, Matrix4 as Matrix47, Vector3 as Vector39, Quaternion as Quaternion3 } from "three";

// node_modules/3d-tiles-renderer/src/plugins/three/fade/FadeManager.js
import { MathUtils as MathUtils5 } from "three";
var { clamp } = MathUtils5;
var FadeManager = class {
  constructor() {
    this.duration = 250;
    this.fadeCount = 0;
    this._lastTick = -1;
    this._fadeState = /* @__PURE__ */ new Map();
    this._fadeParams = /* @__PURE__ */ new WeakMap();
    this.onFadeComplete = null;
    this.onFadeStart = null;
    this.onFadeSetComplete = null;
    this.onFadeSetStart = null;
  }
  // initialize materials in the object
  prepareObject(object) {
    object.traverse((child) => {
      if (child.material) {
        this.prepareMaterial(child.material);
      }
    });
  }
  // delete the object from the fade, reset the material data
  deleteObject(object) {
    if (!object) {
      return;
    }
    this.completeFade(object);
    const fadeParams = this._fadeParams;
    object.traverse((child) => {
      const material = child.material;
      if (material) {
        fadeParams.delete(material);
        material.onBeforeCompile = () => {
        };
        material.needsUpdate = true;
      }
    });
  }
  // initialize the material
  prepareMaterial(material) {
    const fadeParams = this._fadeParams;
    if (fadeParams.has(material)) {
      return;
    }
    const params = {
      fadeIn: { value: 0 },
      fadeOut: { value: 0 }
    };
    material.defines = {
      FEATURE_FADE: 0
    };
    material.onBeforeCompile = (shader) => {
      shader.uniforms = {
        ...shader.uniforms,
        ...params
      };
      shader.fragmentShader = shader.fragmentShader.replace(/void main\(/, (value) => (
        /* glsl */
        `
					#if FEATURE_FADE

					// adapted from https://www.shadertoy.com/view/Mlt3z8
					float bayerDither2x2( vec2 v ) {

						return mod( 3.0 * v.y + 2.0 * v.x, 4.0 );

					}

					float bayerDither4x4( vec2 v ) {

						vec2 P1 = mod( v, 2.0 );
						vec2 P2 = floor( 0.5 * mod( v, 4.0 ) );
						return 4.0 * bayerDither2x2( P1 ) + bayerDither2x2( P2 );

					}

					uniform float fadeIn;
					uniform float fadeOut;
					#endif

					${value}
				`
      )).replace(/#include <dithering_fragment>/, (value) => (
        /* glsl */
        `

					${value}

					#if FEATURE_FADE

					float bayerValue = bayerDither4x4( floor( mod( gl_FragCoord.xy, 4.0 ) ) );
					float bayerBins = 16.0;
					float dither = ( 0.5 + bayerValue ) / bayerBins;
					if ( dither >= fadeIn ) {

						discard;

					}

					if ( dither < fadeOut ) {

						discard;

					}

					#endif

				`
      ));
    };
    fadeParams.set(material, params);
  }
  // Ensure we're storing a fade timer for the provided object
  // Returns whether a new state had to be added
  guaranteeState(object) {
    const fadeState = this._fadeState;
    if (fadeState.has(object)) {
      return false;
    }
    const state = {
      fadeInTarget: 0,
      fadeOutTarget: 0,
      fadeIn: 0,
      fadeOut: 0
    };
    fadeState.set(object, state);
    const fadeParams = this._fadeParams;
    object.traverse((child) => {
      const material = child.material;
      if (material && fadeParams.has(material)) {
        const params = fadeParams.get(material);
        params.fadeIn.value = 0;
        params.fadeOut.value = 0;
      }
    });
    return true;
  }
  // Force the fade to complete in the direction it is already trending
  completeFade(object) {
    const fadeState = this._fadeState;
    if (!fadeState.has(object)) return;
    fadeState.delete(object);
    object.traverse((child) => {
      const material = child.material;
      if (material && material.defines.FEATURE_FADE !== 0) {
        material.defines.FEATURE_FADE = 0;
        material.needsUpdate = true;
      }
    });
    this.fadeCount--;
    if (this.onFadeComplete) {
      this.onFadeComplete(object);
    }
    if (this.fadeCount === 0 && this.onFadeSetComplete) {
      this.onFadeSetComplete();
    }
  }
  completeAllFades() {
    this._fadeState.forEach((value, key) => {
      this.completeFade(key);
    });
  }
  forEachObject(cb) {
    this._fadeState.forEach((info, scene) => cb(scene));
  }
  // Fade the object in
  fadeIn(object) {
    const noState = this.guaranteeState(object);
    const state = this._fadeState.get(object);
    state.fadeInTarget = 1;
    state.fadeOutTarget = 0;
    state.fadeOut = 0;
    if (noState) {
      this.fadeCount++;
      if (this.fadeCount === 1 && this.onFadeSetStart) {
        this.onFadeSetStart();
      }
      if (this.onFadeStart) {
        this.onFadeStart(object);
      }
    }
  }
  // Fade the object out
  fadeOut(object) {
    const noState = this.guaranteeState(object);
    const state = this._fadeState.get(object);
    state.fadeOutTarget = 1;
    if (noState) {
      state.fadeInTarget = 1;
      state.fadeIn = 1;
      this.fadeCount++;
      if (this.fadeCount === 1 && this.onFadeSetStart) {
        this.onFadeSetStart();
      }
      if (this.onFadeStart) {
        this.onFadeStart(object);
      }
    }
  }
  // Tick the fade timer for each actively fading object
  update() {
    const time = window.performance.now();
    if (this._lastTick === -1) {
      this._lastTick = time;
    }
    const delta = clamp((time - this._lastTick) / this.duration, 0, 1);
    this._lastTick = time;
    const fadeState = this._fadeState;
    const fadeParams = this._fadeParams;
    fadeState.forEach((state, object) => {
      const {
        fadeOutTarget,
        fadeInTarget
      } = state;
      let {
        fadeOut,
        fadeIn
      } = state;
      const fadeInSign = Math.sign(fadeInTarget - fadeIn);
      fadeIn = clamp(fadeIn + fadeInSign * delta, 0, 1);
      const fadeOutSign = Math.sign(fadeOutTarget - fadeOut);
      fadeOut = clamp(fadeOut + fadeOutSign * delta, 0, 1);
      state.fadeIn = fadeIn;
      state.fadeOut = fadeOut;
      const defineValue = Number(fadeOut !== fadeOutTarget || fadeIn !== fadeInTarget);
      object.traverse((child) => {
        const material = child.material;
        if (material && fadeParams.has(material)) {
          const uniforms = fadeParams.get(material);
          uniforms.fadeIn.value = fadeIn;
          uniforms.fadeOut.value = fadeOut;
          if (defineValue !== material.defines.FEATURE_FADE) {
            material.defines.FEATURE_FADE = defineValue;
            material.needsUpdate = true;
          }
        }
      });
      const fadeOutComplete = fadeOut === 1 || fadeOut === 0;
      const fadeInComplete = fadeIn === 1 || fadeIn === 0;
      if (fadeOutComplete && fadeInComplete || fadeOut >= fadeIn) {
        this.completeFade(object);
      }
    });
  }
};

// node_modules/3d-tiles-renderer/src/plugins/three/fade/TilesFadePlugin.js
var HAS_POPPED_IN = /* @__PURE__ */ Symbol("HAS_POPPED_IN");
var _fromPos = new Vector39();
var _toPos = new Vector39();
var _fromQuat = new Quaternion3();
var _toQuat = new Quaternion3();
var _scale = new Vector39();
function onTileVisibilityChange(scene, tile, visible) {
  scene.visible = true;
  if (!visible) {
    this._fadeGroup.add(scene);
    this._fadeManager.fadeOut(scene);
  } else {
    const isRootRenderableTile = tile.__depthFromRenderedParent === 1;
    if (isRootRenderableTile) {
      if (tile[HAS_POPPED_IN] || this.fadeRootTiles) {
        this._fadeManager.fadeIn(scene);
      }
      tile[HAS_POPPED_IN] = true;
    } else {
      this._fadeManager.fadeIn(scene);
    }
  }
}
function onLoadModel(scene, tile) {
  this._fadeManager.prepareObject(scene);
  this._tileMap.set(scene, tile);
}
function onDisposeModel(scene) {
  this._fadeManager.deleteObject(scene);
  this._tileMap.delete(scene);
}
function onFadeComplete(object) {
  if (object.parent === this._fadeGroup) {
    this._fadeGroup.remove(object);
  }
}
function onAddCamera(camera) {
  this._prevCameraTransforms.set(camera, new Matrix47());
}
function onDeleteCamera(camera) {
  this._prevCameraTransforms.delete(camera);
}
function onUpdateBefore() {
  const fadeManager = this._fadeManager;
  const tiles = this.tiles;
  this._fadingBefore = fadeManager.fadeCount;
  this._displayActiveTiles = tiles.displayActiveTiles;
  tiles.displayActiveTiles = true;
}
function onUpdateAfter() {
  const fadeManager = this._fadeManager;
  const fadeGroup = this._fadeGroup;
  const displayActiveTiles = this._displayActiveTiles;
  const fadingBefore = this._fadingBefore;
  const tiles = this.tiles;
  const prevCameraTransforms = this._prevCameraTransforms;
  const tileMap = this._tileMap;
  const lruCache = tiles.lruCache;
  const cameras = tiles.cameras;
  tiles.displayActiveTiles = displayActiveTiles;
  fadeManager.update();
  const fadingAfter = fadeManager.fadeCount;
  if (fadingBefore !== 0 && fadingAfter !== 0) {
    tiles.dispatchEvent({ type: "fade-change" });
    tiles.dispatchEvent({ type: "force-rerender" });
  }
  if (!displayActiveTiles) {
    tiles.visibleTiles.forEach((t) => {
      t.cached.scene.visible = t.__inFrustum;
    });
  }
  if (this.maximumFadeOutTiles < fadeGroup.children.length) {
    let isMovingFast = true;
    cameras.forEach((camera) => {
      if (!prevCameraTransforms.has(camera)) {
        return;
      }
      const currMatrix = camera.matrixWorld;
      const prevMatrix = prevCameraTransforms.get(camera);
      currMatrix.decompose(_toPos, _toQuat, _scale);
      prevMatrix.decompose(_fromPos, _fromQuat, _scale);
      const angleTo = _toQuat.angleTo(_fromQuat);
      const positionTo = _toPos.distanceTo(_fromPos);
      isMovingFast = isMovingFast && (angleTo > 0.25 || positionTo > 0.1);
    });
    if (isMovingFast) {
      fadeManager.completeAllFades();
    }
  }
  cameras.forEach((camera) => {
    prevCameraTransforms.get(camera).copy(camera.matrixWorld);
  });
  fadeManager.forEachObject((scene) => {
    lruCache.markUsed(tileMap.get(scene));
  });
}
var TilesFadePlugin = class {
  get fadeDuration() {
    return this._fadeManager.duration;
  }
  set fadeDuration(value) {
    this._fadeManager.duration = Number(value);
  }
  get fadingTiles() {
    return this._fadeManager.fadeCount;
  }
  constructor(options) {
    options = {
      maximumFadeOutTiles: 50,
      fadeRootTiles: false,
      fadeDuration: 250,
      ...options
    };
    this.name = "FADE_TILES_PLUGIN";
    this.tiles = null;
    this._fadeManager = new FadeManager();
    this._prevCameraTransforms = null;
    this._fadeGroup = null;
    this._tileMap = null;
    this.maximumFadeOutTiles = options.maximumFadeOutTiles;
    this.fadeRootTiles = options.fadeRootTiles;
    this.fadeDuration = options.fadeDuration;
  }
  init(tiles) {
    const fadeGroup = new Group3();
    fadeGroup.name = "TilesFadeGroup";
    tiles.group.add(fadeGroup);
    const fadeManager = this._fadeManager;
    fadeManager.onFadeSetStart = () => {
      tiles.dispatchEvent({ type: "fade-start" });
      tiles.dispatchEvent({ type: "force-rerender" });
    };
    fadeManager.onFadeSetComplete = () => {
      tiles.dispatchEvent({ type: "fade-end" });
      tiles.dispatchEvent({ type: "force-rerender" });
    };
    fadeManager.onFadeComplete = onFadeComplete.bind(this);
    this.tiles = tiles;
    this._fadeManager = fadeManager;
    this._fadeGroup = fadeGroup;
    this._tileMap = /* @__PURE__ */ new Map();
    this._prevCameraTransforms = /* @__PURE__ */ new Map();
    tiles.cameras.forEach((camera) => {
      this._prevCameraTransforms.set(camera, new Matrix47());
    });
    tiles.forEachLoadedModel((scene, tile) => {
      onLoadModel.call(this, scene, tile);
    });
    this._onLoadModel = (e) => onLoadModel.call(this, e.scene, e.tile);
    this._onDisposeModel = (e) => onDisposeModel.call(this, e.scene);
    this._onTileVisibilityChange = (e) => onTileVisibilityChange.call(this, e.scene, e.tile, e.visible);
    this._onAddCamera = (e) => onAddCamera.call(this, e.camera);
    this._onDeleteCamera = (e) => onDeleteCamera.call(this, e.camera);
    this._onUpdateBefore = () => onUpdateBefore.call(this);
    this._onUpdateAfter = () => onUpdateAfter.call(this);
    tiles.addEventListener("load-model", this._onLoadModel);
    tiles.addEventListener("dispose-model", this._onDisposeModel);
    tiles.addEventListener("tile-visibility-change", this._onTileVisibilityChange);
    tiles.addEventListener("add-camera", this._onAddCamera);
    tiles.addEventListener("delete-camera", this._onDeleteCamera);
    tiles.addEventListener("update-before", this._onUpdateBefore);
    tiles.addEventListener("update-after", this._onUpdateAfter);
  }
  dispose() {
    const tiles = this.tiles;
    tiles.removeEventListener("load-model", this._onLoadModel);
    tiles.removeEventListener("dispose-model", this._onDisposeModel);
    tiles.removeEventListener("tile-visibility-change", this._onTileVisibilityChange);
    tiles.removeEventListener("add-camera", this._onAddCamera);
    tiles.removeEventListener("delete-camera", this._onDeleteCamera);
    tiles.removeEventListener("update-before", this._onUpdateBefore);
    tiles.removeEventListener("update-after", this._onUpdateAfter);
    tiles.forEachLoadedModel((scene) => {
      this._fadeManager.deleteObject(scene);
      this._tileMap.delete(scene);
      scene.visible = true;
    });
    this._fadeGroup.removeFromParent();
  }
};

// node_modules/3d-tiles-renderer/src/three/math/EllipsoidRegion.js
import { MathUtils as MathUtils6, Matrix4 as Matrix48 } from "three";
import { Vector3 as Vector310 } from "three";
var PI = Math.PI;
var HALF_PI = PI / 2;
var _orthoX = new Vector310();
var _orthoY = new Vector310();
var _orthoZ = new Vector310();
var _invMatrix = new Matrix48();
var _poolIndex = 0;
var _pointsPool = [];
function getVector(usePool = false) {
  if (!usePool) {
    return new Vector310();
  }
  if (!_pointsPool[_poolIndex]) {
    _pointsPool[_poolIndex] = new Vector310();
  }
  _poolIndex++;
  return _pointsPool[_poolIndex - 1];
}
function resetPool() {
  _poolIndex = 0;
}
var EllipsoidRegion = class extends Ellipsoid {
  constructor(x, y, z, latStart = -HALF_PI, latEnd = HALF_PI, lonStart = 0, lonEnd = 2 * PI, heightStart = 0, heightEnd = 0) {
    super(x, y, z);
    this.latStart = latStart;
    this.latEnd = latEnd;
    this.lonStart = lonStart;
    this.lonEnd = lonEnd;
    this.heightStart = heightStart;
    this.heightEnd = heightEnd;
  }
  _getPoints(usePool = false) {
    const {
      latStart,
      latEnd,
      lonStart,
      lonEnd,
      heightStart,
      heightEnd
    } = this;
    const midLat = MathUtils6.mapLinear(0.5, 0, 1, latStart, latEnd);
    const midLon = MathUtils6.mapLinear(0.5, 0, 1, lonStart, lonEnd);
    const lonOffset = Math.floor(lonStart / HALF_PI) * HALF_PI;
    const latlon = [
      [-PI / 2, 0],
      [PI / 2, 0],
      [0, lonOffset],
      [0, lonOffset + PI / 2],
      [0, lonOffset + PI],
      [0, lonOffset + 3 * PI / 2],
      [latStart, lonEnd],
      [latEnd, lonEnd],
      [latStart, lonStart],
      [latEnd, lonStart],
      [0, lonStart],
      [0, lonEnd],
      [midLat, midLon],
      [latStart, midLon],
      [latEnd, midLon],
      [midLat, lonStart],
      [midLat, lonEnd]
    ];
    const target = [];
    const total = latlon.length;
    for (let z = 0; z <= 1; z++) {
      const height = MathUtils6.mapLinear(z, 0, 1, heightStart, heightEnd);
      for (let i = 0, l = total; i < l; i++) {
        const [lat, lon] = latlon[i];
        if (lat >= latStart && lat <= latEnd && lon >= lonStart && lon <= lonEnd) {
          const v = getVector(usePool);
          target.push(v);
          this.getCartographicToPosition(lat, lon, height, v);
        }
      }
    }
    return target;
  }
  getBoundingBox(box, matrix) {
    resetPool();
    const {
      latStart,
      latEnd,
      lonStart,
      lonEnd
    } = this;
    const latRange = latEnd - latStart;
    if (latRange < PI / 2) {
      const midLat = MathUtils6.mapLinear(0.5, 0, 1, latStart, latEnd);
      const midLon = MathUtils6.mapLinear(0.5, 0, 1, lonStart, lonEnd);
      this.getCartographicToNormal(midLat, midLon, _orthoZ);
      _orthoY.set(0, 0, 1);
      _orthoX.crossVectors(_orthoY, _orthoZ);
      _orthoY.crossVectors(_orthoX, _orthoZ);
      matrix.makeBasis(_orthoX, _orthoY, _orthoZ);
    } else {
      _orthoX.set(1, 0, 0);
      _orthoY.set(0, 1, 0);
      _orthoZ.set(0, 0, 1);
      matrix.makeBasis(_orthoX, _orthoY, _orthoZ);
    }
    _invMatrix.copy(matrix).invert();
    const points = this._getPoints(true);
    for (let i = 0, l = points.length; i < l; i++) {
      points[i].applyMatrix4(_invMatrix);
    }
    box.makeEmpty();
    box.setFromPoints(points);
  }
  getBoundingSphere(sphere, center) {
    resetPool();
    const points = this._getPoints(true);
    sphere.makeEmpty();
    sphere.setFromPoints(points, center);
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/gltf/GLTFCesiumRTCExtension.js
var GLTFCesiumRTCExtension2 = class extends GLTFCesiumRTCExtension {
  constructor(...args) {
    super(...args);
    console.warn('GLTFCesiumRTCExtension: Plugins should now be imported from "3d-tiles-renderer/plugins" path.');
  }
};

// node_modules/3d-tiles-renderer/src/three/loaders/GLTFExtensionLoader.js
var GLTFExtensionLoader = class extends LoaderBase {
  constructor(manager = DefaultLoadingManager5) {
    super();
    this.manager = manager;
  }
  parse(buffer) {
    return new Promise((resolve, reject) => {
      const manager = this.manager;
      const fetchOptions = this.fetchOptions;
      let loader = manager.getHandler("path.gltf") || manager.getHandler("path.glb");
      if (!loader) {
        loader = new GLTFLoader(manager);
        loader.register(() => new GLTFCesiumRTCExtension2());
      }
      if (fetchOptions.credentials === "include" && fetchOptions.mode === "cors") {
        loader.setCrossOrigin("use-credentials");
      }
      if ("credentials" in fetchOptions) {
        loader.setWithCredentials(fetchOptions.credentials === "include");
      }
      if (fetchOptions.headers) {
        loader.setRequestHeader(fetchOptions.headers);
      }
      let resourcePath = loader.resourcePath || loader.path || this.workingPath;
      if (!/[\\/]$/.test(resourcePath) && resourcePath.length) {
        resourcePath += "/";
      }
      loader.parse(buffer, resourcePath, (model) => {
        resolve(model);
      }, reject);
    });
  }
};

// node_modules/3d-tiles-renderer/src/three/TilesGroup.js
import { Group as Group4, Matrix4 as Matrix49 } from "three";
var tempMat3 = new Matrix49();
var TilesGroup = class extends Group4 {
  constructor(tilesRenderer) {
    super();
    this.name = "TilesRenderer.TilesGroup";
    this.tilesRenderer = tilesRenderer;
  }
  raycast(raycaster, intersects) {
    if (this.tilesRenderer.optimizeRaycast) {
      this.tilesRenderer.raycast(raycaster, intersects);
      return false;
    }
    return true;
  }
  updateMatrixWorld(force) {
    if (this.matrixAutoUpdate) {
      this.updateMatrix();
    }
    if (this.matrixWorldNeedsUpdate || force) {
      if (this.parent === null) {
        tempMat3.copy(this.matrix);
      } else {
        tempMat3.multiplyMatrices(this.parent.matrixWorld, this.matrix);
      }
      this.matrixWorldNeedsUpdate = false;
      const elA = tempMat3.elements;
      const elB = this.matrixWorld.elements;
      let isDifferent = false;
      for (let i = 0; i < 16; i++) {
        const itemA = elA[i];
        const itemB = elB[i];
        const diff = Math.abs(itemA - itemB);
        if (diff > Number.EPSILON) {
          isDifferent = true;
          break;
        }
      }
      if (isDifferent) {
        this.matrixWorld.copy(tempMat3);
        const children = this.children;
        for (let i = 0, l = children.length; i < l; i++) {
          children[i].updateMatrixWorld();
        }
      }
    }
  }
};

// node_modules/3d-tiles-renderer/src/three/TilesRenderer.js
import {
  Matrix4 as Matrix412,
  Vector3 as Vector315,
  Vector2 as Vector27,
  Euler as Euler2,
  LoadingManager,
  EventDispatcher,
  REVISION as REVISION3
} from "three";

// node_modules/3d-tiles-renderer/src/three/raycastTraverse.js
import { Matrix4 as Matrix410, Ray as Ray2, Vector3 as Vector311, REVISION as REVISION2 } from "three";
var REVISION_LESS_165 = parseInt(REVISION2) < 165;
var _mat = new Matrix410();
var _localRay = new Ray2();
var _vec5 = new Vector311();
var _hitArray = [];
function distanceSort(a, b) {
  return a.distance - b.distance;
}
function intersectTileScene(tile, raycaster, renderer, intersects) {
  const { scene } = tile.cached;
  if (REVISION_LESS_165) {
    scene.traverse((c) => {
      Object.getPrototypeOf(c).raycast.call(c, raycaster, intersects);
    });
  } else {
    const didRaycast = renderer.invokeOnePlugin((plugin) => plugin.raycastTile && plugin.raycastTile(tile, scene, raycaster, intersects));
    if (!didRaycast) {
      raycaster.intersectObject(scene, true, intersects);
    }
  }
}
function intersectTileSceneFirstHist(tile, raycaster, renderer) {
  intersectTileScene(tile, raycaster, renderer, _hitArray);
  _hitArray.sort(distanceSort);
  const hit = _hitArray[0] || null;
  _hitArray.length = 0;
  return hit;
}
function raycastTraverseFirstHit(renderer, tile, raycaster, localRay = null) {
  const { group, activeTiles } = renderer;
  renderer.ensureChildrenArePreprocessed(tile);
  if (localRay === null) {
    localRay = _localRay;
    _mat.copy(group.matrixWorld).invert();
    localRay.copy(raycaster.ray).applyMatrix4(_mat);
  }
  const array = [];
  const children = tile.children;
  for (let i = 0, l = children.length; i < l; i++) {
    const child = children[i];
    if (!child.__used) {
      continue;
    }
    const boundingVolume = child.cached.boundingVolume;
    if (boundingVolume.intersectRay(localRay, _vec5) !== null) {
      _vec5.applyMatrix4(group.matrixWorld);
      array.push({
        distance: _vec5.distanceToSquared(raycaster.ray.origin),
        tile: child
      });
    }
  }
  array.sort(distanceSort);
  let bestHit = null;
  let bestHitDistSq = Infinity;
  if (activeTiles.has(tile)) {
    const hit = intersectTileSceneFirstHist(tile, raycaster, renderer);
    if (hit) {
      bestHit = hit;
      bestHitDistSq = hit.distance * hit.distance;
    }
  }
  for (let i = 0, l = array.length; i < l; i++) {
    const data = array[i];
    const boundingVolumeDistSq = data.distance;
    const tile2 = data.tile;
    if (boundingVolumeDistSq > bestHitDistSq) {
      break;
    }
    const hit = raycastTraverseFirstHit(renderer, tile2, raycaster, localRay);
    if (hit) {
      const hitDistSq = hit.distance * hit.distance;
      if (hitDistSq < bestHitDistSq) {
        bestHit = hit;
        bestHitDistSq = hitDistSq;
      }
    }
  }
  return bestHit;
}
function raycastTraverse(renderer, tile, raycaster, intersects, localRay = null) {
  const { group, activeTiles } = renderer;
  const { boundingVolume } = tile.cached;
  renderer.ensureChildrenArePreprocessed(tile);
  if (localRay === null) {
    localRay = _localRay;
    _mat.copy(group.matrixWorld).invert();
    localRay.copy(raycaster.ray).applyMatrix4(_mat);
  }
  if (!tile.__used || !boundingVolume.intersectsRay(localRay)) {
    return;
  }
  if (activeTiles.has(tile)) {
    intersectTileScene(tile, raycaster, renderer, intersects);
  }
  const children = tile.children;
  for (let i = 0, l = children.length; i < l; i++) {
    raycastTraverse(renderer, children[i], raycaster, intersects, localRay);
  }
}

// node_modules/3d-tiles-renderer/src/three/math/TileBoundingVolume.js
import { Vector3 as Vector313, Sphere as Sphere3 } from "three";

// node_modules/3d-tiles-renderer/src/three/math/OBB.js
import { Matrix4 as Matrix411, Box3 as Box32, Vector3 as Vector312, Plane, Ray as Ray3 } from "three";
var _worldMin = new Vector312();
var _worldMax = new Vector312();
var _norm2 = new Vector312();
var _ray2 = new Ray3();
var OBB = class {
  constructor(box = new Box32(), transform = new Matrix411()) {
    this.box = box.clone();
    this.transform = transform.clone();
    this.inverseTransform = new Matrix411();
    this.points = new Array(8).fill().map(() => new Vector312());
    this.planes = new Array(6).fill().map(() => new Plane());
  }
  /**
   * Clamps the given point within the bounds of this OBB
   * @param {Vector3} point
   * @param {Vector3} result
   * @returns {Vector3}
   */
  clampPoint(point, result) {
    return result.copy(point).applyMatrix4(this.inverseTransform).clamp(this.box.min, this.box.max).applyMatrix4(this.transform);
  }
  /**
   * Returns the distance from any edge of this OBB to the specified point.
   * If the point lies inside of this box, the distance will be 0.
   * @param {Vector3} point
   * @returns {number}
   */
  distanceToPoint(point) {
    return this.clampPoint(point, _norm2).distanceTo(point);
  }
  containsPoint(point) {
    _norm2.copy(point).applyMatrix4(this.inverseTransform);
    return this.box.containsPoint(_norm2);
  }
  // returns boolean indicating whether the ray has intersected the obb
  intersectsRay(ray) {
    _ray2.copy(ray).applyMatrix4(this.inverseTransform);
    return _ray2.intersectsBox(this.box);
  }
  // Sets "target" equal to the intersection point.
  // Returns "null" if no intersection found.
  intersectRay(ray, target) {
    _ray2.copy(ray).applyMatrix4(this.inverseTransform);
    if (_ray2.intersectBox(this.box, target)) {
      target.applyMatrix4(this.transform);
      return target;
    } else {
      return null;
    }
  }
  update() {
    const { points, inverseTransform, transform, box } = this;
    inverseTransform.copy(transform).invert();
    const { min, max } = box;
    let index = 0;
    for (let x = -1; x <= 1; x += 2) {
      for (let y = -1; y <= 1; y += 2) {
        for (let z = -1; z <= 1; z += 2) {
          points[index].set(
            x < 0 ? min.x : max.x,
            y < 0 ? min.y : max.y,
            z < 0 ? min.z : max.z
          ).applyMatrix4(transform);
          index++;
        }
      }
    }
    this.updatePlanes();
  }
  updatePlanes() {
    _worldMin.copy(this.box.min).applyMatrix4(this.transform);
    _worldMax.copy(this.box.max).applyMatrix4(this.transform);
    _norm2.set(0, 0, 1).transformDirection(this.transform);
    this.planes[0].setFromNormalAndCoplanarPoint(_norm2, _worldMin);
    this.planes[1].setFromNormalAndCoplanarPoint(_norm2, _worldMax).negate();
    _norm2.set(0, 1, 0).transformDirection(this.transform);
    this.planes[2].setFromNormalAndCoplanarPoint(_norm2, _worldMin);
    this.planes[3].setFromNormalAndCoplanarPoint(_norm2, _worldMax).negate();
    _norm2.set(1, 0, 0).transformDirection(this.transform);
    this.planes[4].setFromNormalAndCoplanarPoint(_norm2, _worldMin);
    this.planes[5].setFromNormalAndCoplanarPoint(_norm2, _worldMax).negate();
  }
  // based on three.js' Box3 "intersects frustum" function
  intersectsFrustum(frustum) {
    const { points } = this;
    const { planes } = frustum;
    for (let i = 0; i < 6; i++) {
      const plane = planes[i];
      let maxDistance = -Infinity;
      for (let j = 0; j < 8; j++) {
        const v = points[j];
        const dist = plane.distanceToPoint(v);
        maxDistance = maxDistance < dist ? dist : maxDistance;
      }
      if (maxDistance < 0) {
        return false;
      }
    }
    for (let i = 0; i < 6; i++) {
      const plane = this.planes[i];
      let maxDistance = -Infinity;
      for (let j = 0; j < 8; j++) {
        const v = frustum.points[j];
        const dist = plane.distanceToPoint(v);
        maxDistance = maxDistance < dist ? dist : maxDistance;
      }
      if (maxDistance < 0) {
        return false;
      }
    }
    return true;
  }
};

// node_modules/3d-tiles-renderer/src/three/math/TileBoundingVolume.js
var _vecX2 = new Vector313();
var _vecY2 = new Vector313();
var _vecZ2 = new Vector313();
var _sphereVec = new Vector313();
var _obbVec = new Vector313();
var TileBoundingVolume = class {
  constructor() {
    this.sphere = null;
    this.obb = null;
    this.region = null;
    this.regionObb = null;
  }
  intersectsRay(ray) {
    const sphere = this.sphere;
    const obb = this.obb || this.regionObb;
    if (sphere && !ray.intersectsSphere(sphere)) {
      return false;
    }
    if (obb && !obb.intersectsRay(ray)) {
      return false;
    }
    return true;
  }
  intersectRay(ray, target = null) {
    const sphere = this.sphere;
    const obb = this.obb || this.regionObb;
    let sphereDistSq = -Infinity;
    let obbDistSq = -Infinity;
    if (sphere) {
      if (ray.intersectSphere(sphere, _sphereVec)) {
        sphereDistSq = sphere.containsPoint(ray.origin) ? 0 : ray.origin.distanceToSquared(_sphereVec);
      }
    }
    if (obb) {
      if (obb.intersectRay(ray, _obbVec)) {
        obbDistSq = obb.containsPoint(ray.origin) ? 0 : ray.origin.distanceToSquared(_obbVec);
      }
    }
    const furthestDist = Math.max(sphereDistSq, obbDistSq);
    if (furthestDist === -Infinity) {
      return null;
    }
    ray.at(Math.sqrt(furthestDist), target);
    return target;
  }
  distanceToPoint(point) {
    const sphere = this.sphere;
    const obb = this.obb || this.regionObb;
    let sphereDistance = -Infinity;
    let obbDistance = -Infinity;
    if (sphere) {
      sphereDistance = Math.max(sphere.distanceToPoint(point), 0);
    }
    if (obb) {
      obbDistance = obb.distanceToPoint(point);
    }
    return sphereDistance > obbDistance ? sphereDistance : obbDistance;
  }
  intersectsFrustum(frustum) {
    const obb = this.obb || this.regionObb;
    const sphere = this.sphere;
    if (sphere && !frustum.intersectsSphere(sphere)) {
      return false;
    }
    if (obb && !obb.intersectsFrustum(frustum)) {
      return false;
    }
    return Boolean(sphere || obb);
  }
  getOBB(targetBox, targetMatrix) {
    const obb = this.obb || this.regionObb;
    if (obb) {
      targetBox.copy(obb.box);
      targetMatrix.copy(obb.transform);
    } else {
      this.getAABB(targetBox);
      targetMatrix.identity();
    }
  }
  getAABB(target) {
    if (this.sphere) {
      this.sphere.getBoundingBox(target);
    } else {
      const obb = this.obb || this.regionObb;
      target.copy(obb.box).applyMatrix4(obb.transform);
    }
  }
  getSphere(target) {
    if (this.sphere) {
      target.copy(this.sphere);
    } else if (this.region) {
      this.region.getBoundingSphere(target);
    } else {
      const obb = this.obb || this.regionObb;
      obb.box.getBoundingSphere(target);
      target.applyMatrix4(obb.transform);
    }
  }
  setObbData(data, transform) {
    const obb = new OBB();
    _vecX2.set(data[3], data[4], data[5]);
    _vecY2.set(data[6], data[7], data[8]);
    _vecZ2.set(data[9], data[10], data[11]);
    const scaleX = _vecX2.length();
    const scaleY = _vecY2.length();
    const scaleZ = _vecZ2.length();
    _vecX2.normalize();
    _vecY2.normalize();
    _vecZ2.normalize();
    if (scaleX === 0) {
      _vecX2.crossVectors(_vecY2, _vecZ2);
    }
    if (scaleY === 0) {
      _vecY2.crossVectors(_vecX2, _vecZ2);
    }
    if (scaleZ === 0) {
      _vecZ2.crossVectors(_vecX2, _vecY2);
    }
    obb.transform.set(
      _vecX2.x,
      _vecY2.x,
      _vecZ2.x,
      data[0],
      _vecX2.y,
      _vecY2.y,
      _vecZ2.y,
      data[1],
      _vecX2.z,
      _vecY2.z,
      _vecZ2.z,
      data[2],
      0,
      0,
      0,
      1
    ).premultiply(transform);
    obb.box.min.set(-scaleX, -scaleY, -scaleZ);
    obb.box.max.set(scaleX, scaleY, scaleZ);
    obb.update();
    this.obb = obb;
  }
  setSphereData(x, y, z, radius, transform) {
    const sphere = new Sphere3();
    sphere.center.set(x, y, z);
    sphere.radius = radius;
    sphere.applyMatrix4(transform);
    this.sphere = sphere;
  }
  setRegionData(ellipsoid, west, south, east, north, minHeight, maxHeight) {
    const region = new EllipsoidRegion(
      ...ellipsoid.radius,
      south,
      north,
      west,
      east,
      minHeight,
      maxHeight
    );
    const obb = new OBB();
    region.getBoundingBox(obb.box, obb.transform);
    obb.update();
    this.region = region;
    this.regionObb = obb;
  }
};

// node_modules/3d-tiles-renderer/src/three/math/ExtendedFrustum.js
import { Frustum, Matrix3 as Matrix32, Vector3 as Vector314 } from "three";
var _mat3 = new Matrix32();
function findIntersectionPoint(plane1, plane2, plane3, target) {
  const A = _mat3.set(
    plane1.normal.x,
    plane1.normal.y,
    plane1.normal.z,
    plane2.normal.x,
    plane2.normal.y,
    plane2.normal.z,
    plane3.normal.x,
    plane3.normal.y,
    plane3.normal.z
  );
  target.set(-plane1.constant, -plane2.constant, -plane3.constant);
  target.applyMatrix3(A.invert());
  return target;
}
var ExtendedFrustum = class extends Frustum {
  constructor() {
    super();
    this.points = Array(8).fill().map(() => new Vector314());
  }
  setFromProjectionMatrix(m, coordinateSystem) {
    super.setFromProjectionMatrix(m, coordinateSystem);
    this.calculateFrustumPoints();
  }
  calculateFrustumPoints() {
    const { planes, points } = this;
    const planeIntersections = [
      [planes[0], planes[3], planes[4]],
      // Near top left
      [planes[1], planes[3], planes[4]],
      // Near top right
      [planes[0], planes[2], planes[4]],
      // Near bottom left
      [planes[1], planes[2], planes[4]],
      // Near bottom right
      [planes[0], planes[3], planes[5]],
      // Far top left
      [planes[1], planes[3], planes[5]],
      // Far top right
      [planes[0], planes[2], planes[5]],
      // Far bottom left
      [planes[1], planes[2], planes[5]]
      // Far bottom right
    ];
    planeIntersections.forEach((planes2, index) => {
      findIntersectionPoint(planes2[0], planes2[1], planes2[2], points[index]);
    });
  }
};

// node_modules/3d-tiles-renderer/src/three/utilities.js
import * as THREE from "three";
function estimateBytesUsed2(object) {
  const { TextureUtils } = THREE;
  if (!TextureUtils) {
    return 0;
  }
  const dedupeSet = /* @__PURE__ */ new Set();
  let totalBytes = 0;
  object.traverse((c) => {
    if (c.geometry && !dedupeSet.has(c.geometry)) {
      totalBytes += estimateBytesUsed(c.geometry);
      dedupeSet.add(c.geometry);
    }
    if (c.material) {
      const material = c.material;
      for (const key in material) {
        const value = material[key];
        if (value && value.isTexture && !dedupeSet.has(value)) {
          const { format, type, image } = value;
          const { width, height } = image;
          const bytes = TextureUtils.getByteLength(width, height, format, type);
          totalBytes += value.generateMipmaps ? bytes * 4 / 3 : bytes;
          dedupeSet.add(value);
        }
      }
    }
  });
  return totalBytes;
}

// node_modules/3d-tiles-renderer/src/three/TilesRenderer.js
var _mat2 = new Matrix412();
var _euler2 = new Euler2();
var REVISION_LESS_1652 = parseInt(REVISION3) < 165;
var INITIAL_FRUSTUM_CULLED = /* @__PURE__ */ Symbol("INITIAL_FRUSTUM_CULLED");
var tempMat4 = new Matrix412();
var tempMat22 = new Matrix412();
var tempVector = new Vector315();
var tempVector2 = new Vector27();
var X_AXIS = new Vector315(1, 0, 0);
var Y_AXIS = new Vector315(0, 1, 0);
function updateFrustumCulled(object, toInitialValue) {
  object.traverse((c) => {
    c.frustumCulled = c[INITIAL_FRUSTUM_CULLED] && toInitialValue;
  });
}
var TilesRenderer = class extends TilesRendererBase {
  get autoDisableRendererCulling() {
    return this._autoDisableRendererCulling;
  }
  set autoDisableRendererCulling(value) {
    if (this._autoDisableRendererCulling !== value) {
      super._autoDisableRendererCulling = value;
      this.forEachLoadedModel((scene) => {
        updateFrustumCulled(scene, !value);
      });
    }
  }
  constructor(...args) {
    super(...args);
    this.group = new TilesGroup(this);
    this.ellipsoid = WGS84_ELLIPSOID.clone();
    this.cameras = [];
    this.cameraMap = /* @__PURE__ */ new Map();
    this.cameraInfo = [];
    this.activeTiles = /* @__PURE__ */ new Set();
    this.visibleTiles = /* @__PURE__ */ new Set();
    this.optimizeRaycast = true;
    this._eventDispatcher = new EventDispatcher();
    this._upRotationMatrix = new Matrix412();
    this.lruCache.computeMemoryUsageCallback = (tile) => tile.cached.bytesUsed ?? null;
    this._autoDisableRendererCulling = true;
    this._loadingTiles = false;
    const manager = new LoadingManager();
    manager.setURLModifier((url) => {
      if (this.preprocessURL) {
        return this.preprocessURL(url);
      } else {
        return url;
      }
    });
    this.manager = manager;
    if (REVISION_LESS_1652) {
      const tilesRenderer = this;
      this._overridenRaycast = function(raycaster, intersects) {
        if (!tilesRenderer.optimizeRaycast) {
          Object.getPrototypeOf(this).raycast.call(this, raycaster, intersects);
        }
      };
    }
  }
  addEventListener(...args) {
    this._eventDispatcher.addEventListener(...args);
  }
  hasEventListener(...args) {
    this._eventDispatcher.hasEventListener(...args);
  }
  removeEventListener(...args) {
    this._eventDispatcher.removeEventListener(...args);
  }
  dispatchEvent(...args) {
    this._eventDispatcher.dispatchEvent(...args);
  }
  /* Public API */
  getBounds(...args) {
    console.warn("TilesRenderer: getBounds has been renamed to getBoundingBox.");
    return this.getBoundingBox(...args);
  }
  getOrientedBounds(...args) {
    console.warn("TilesRenderer: getOrientedBounds has been renamed to getOrientedBoundingBox.");
    return this.getOrientedBoundingBox(...args);
  }
  getBoundingBox(target) {
    if (!this.root) {
      return false;
    }
    const boundingVolume = this.root.cached.boundingVolume;
    if (boundingVolume) {
      boundingVolume.getAABB(target);
      return true;
    } else {
      return false;
    }
  }
  getOrientedBoundingBox(targetBox, targetMatrix) {
    if (!this.root) {
      return false;
    }
    const boundingVolume = this.root.cached.boundingVolume;
    if (boundingVolume) {
      boundingVolume.getOBB(targetBox, targetMatrix);
      return true;
    } else {
      return false;
    }
  }
  getBoundingSphere(target) {
    if (!this.root) {
      return false;
    }
    const boundingVolume = this.root.cached.boundingVolume;
    if (boundingVolume) {
      boundingVolume.getSphere(target);
      return true;
    } else {
      return false;
    }
  }
  forEachLoadedModel(callback) {
    this.traverse((tile) => {
      const scene = tile.cached.scene;
      if (scene) {
        callback(scene, tile);
      }
    });
  }
  raycast(raycaster, intersects) {
    if (!this.root) {
      return;
    }
    if (raycaster.firstHitOnly) {
      const hit = raycastTraverseFirstHit(this, this.root, raycaster);
      if (hit) {
        intersects.push(hit);
      }
    } else {
      raycastTraverse(this, this.root, raycaster, intersects);
    }
  }
  hasCamera(camera) {
    return this.cameraMap.has(camera);
  }
  setCamera(camera) {
    const cameras = this.cameras;
    const cameraMap = this.cameraMap;
    if (!cameraMap.has(camera)) {
      cameraMap.set(camera, new Vector27());
      cameras.push(camera);
      this.dispatchEvent({ type: "add-camera", camera });
      return true;
    }
    return false;
  }
  setResolution(camera, xOrVec, y) {
    const cameraMap = this.cameraMap;
    if (!cameraMap.has(camera)) {
      return false;
    }
    const width = xOrVec.isVector2 ? xOrVec.x : xOrVec;
    const height = xOrVec.isVector2 ? xOrVec.y : y;
    const cameraVec = cameraMap.get(camera);
    if (cameraVec.width !== width || cameraVec.height !== height) {
      cameraVec.set(width, height);
      this.dispatchEvent({ type: "camera-resolution-change" });
    }
    return true;
  }
  setResolutionFromRenderer(camera, renderer) {
    renderer.getSize(tempVector2).multiplyScalar(renderer.getPixelRatio());
    return this.setResolution(camera, tempVector2.x, tempVector2.y);
  }
  deleteCamera(camera) {
    const cameras = this.cameras;
    const cameraMap = this.cameraMap;
    if (cameraMap.has(camera)) {
      const index = cameras.indexOf(camera);
      cameras.splice(index, 1);
      cameraMap.delete(camera);
      this.dispatchEvent({ type: "delete-camera", camera });
      return true;
    }
    return false;
  }
  /* Overriden */
  preprocessTileSet(json, url, tile) {
    super.preprocessTileSet(json, url, tile);
    queueMicrotask(() => {
      this.dispatchEvent({
        type: "load-tile-set",
        tileSet: json,
        url
      });
    });
  }
  loadRootTileSet(...args) {
    return super.loadRootTileSet(...args).then(() => {
      const upAxis = this.rootTileSet.asset && this.rootTileSet.asset.gltfUpAxis || "y";
      switch (upAxis.toLowerCase()) {
        case "x":
          this._upRotationMatrix.makeRotationAxis(Y_AXIS, -Math.PI / 2);
          break;
        case "y":
          this._upRotationMatrix.makeRotationAxis(X_AXIS, Math.PI / 2);
          break;
      }
      this.dispatchEvent({ type: "load-content" });
    }).catch(() => {
    });
  }
  update() {
    let needsUpdate = null;
    this.invokeAllPlugins((plugin) => {
      if (plugin.doTilesNeedUpdate) {
        const res = plugin.doTilesNeedUpdate();
        needsUpdate = needsUpdate === null ? res : needsUpdate || res;
      }
    });
    if (needsUpdate === false) {
      this.dispatchEvent({ type: "update-before" });
      this.dispatchEvent({ type: "update-after" });
      return;
    }
    this.dispatchEvent({ type: "update-before" });
    const group = this.group;
    const cameras = this.cameras;
    const cameraMap = this.cameraMap;
    const cameraInfo = this.cameraInfo;
    if (cameras.length === 0) {
      console.warn("TilesRenderer: no cameras defined. Cannot update 3d tiles.");
      return;
    }
    while (cameraInfo.length > cameras.length) {
      cameraInfo.pop();
    }
    while (cameraInfo.length < cameras.length) {
      cameraInfo.push({
        frustum: new ExtendedFrustum(),
        isOrthographic: false,
        sseDenominator: -1,
        // used if isOrthographic:false
        position: new Vector315(),
        invScale: -1,
        pixelSize: 0
        // used if isOrthographic:true
      });
    }
    tempMat22.copy(group.matrixWorld).invert();
    tempVector.setFromMatrixScale(tempMat22);
    if (Math.abs(Math.max(tempVector.x - tempVector.y, tempVector.x - tempVector.z)) > 1e-6) {
      console.warn("ThreeTilesRenderer : Non uniform scale used for tile which may cause issues when calculating screen space error.");
    }
    for (let i = 0, l = cameraInfo.length; i < l; i++) {
      const camera = cameras[i];
      const info = cameraInfo[i];
      const frustum = info.frustum;
      const position = info.position;
      const resolution = cameraMap.get(camera);
      if (resolution.width === 0 || resolution.height === 0) {
        console.warn("TilesRenderer: resolution for camera error calculation is not set.");
      }
      const projection = camera.projectionMatrix.elements;
      info.isOrthographic = projection[15] === 1;
      if (info.isOrthographic) {
        const w = 2 / projection[0];
        const h = 2 / projection[5];
        info.pixelSize = Math.max(h / resolution.height, w / resolution.width);
      } else {
        info.sseDenominator = 2 / projection[5] / resolution.height;
      }
      tempMat4.copy(group.matrixWorld);
      tempMat4.premultiply(camera.matrixWorldInverse);
      tempMat4.premultiply(camera.projectionMatrix);
      frustum.setFromProjectionMatrix(tempMat4);
      position.set(0, 0, 0);
      position.applyMatrix4(camera.matrixWorld);
      position.applyMatrix4(tempMat22);
    }
    super.update();
    this.dispatchEvent({ type: "update-after" });
  }
  preprocessNode(tile, tileSetDir, parentTile = null) {
    super.preprocessNode(tile, tileSetDir, parentTile);
    const transform = new Matrix412();
    if (tile.transform) {
      const transformArr = tile.transform;
      for (let i = 0; i < 16; i++) {
        transform.elements[i] = transformArr[i];
      }
    }
    if (parentTile) {
      transform.premultiply(parentTile.cached.transform);
    }
    const transformInverse = new Matrix412().copy(transform).invert();
    const boundingVolume = new TileBoundingVolume();
    if ("sphere" in tile.boundingVolume) {
      boundingVolume.setSphereData(...tile.boundingVolume.sphere, transform);
    }
    if ("box" in tile.boundingVolume) {
      boundingVolume.setObbData(tile.boundingVolume.box, transform);
    }
    if ("region" in tile.boundingVolume) {
      boundingVolume.setRegionData(this.ellipsoid, ...tile.boundingVolume.region);
    }
    tile.cached = {
      _loadIndex: 0,
      transform,
      transformInverse,
      active: false,
      boundingVolume,
      metadata: null,
      scene: null,
      geometry: null,
      materials: null,
      textures: null
    };
  }
  async requestTileContents(...args) {
    await super.requestTileContents(...args);
    this.dispatchEvent({ type: "load-content" });
  }
  async parseTile(buffer, tile, extension, uri) {
    const cached = tile.cached;
    cached._loadIndex++;
    const uriSplits = uri.split(/[\\/]/g);
    uriSplits.pop();
    const workingPath = uriSplits.join("/");
    const fetchOptions = this.fetchOptions;
    const manager = this.manager;
    const loadIndex = cached._loadIndex;
    let promise = null;
    const cachedTransform = cached.transform;
    const upRotationMatrix = this._upRotationMatrix;
    const fileType = (readMagicBytes(buffer) || extension).toLowerCase();
    switch (fileType) {
      case "b3dm": {
        const loader = new B3DMLoader(manager);
        loader.workingPath = workingPath;
        loader.fetchOptions = fetchOptions;
        loader.adjustmentTransform.copy(upRotationMatrix);
        promise = loader.parse(buffer);
        break;
      }
      case "pnts": {
        const loader = new PNTSLoader(manager);
        loader.workingPath = workingPath;
        loader.fetchOptions = fetchOptions;
        promise = loader.parse(buffer);
        break;
      }
      case "i3dm": {
        const loader = new I3DMLoader(manager);
        loader.workingPath = workingPath;
        loader.fetchOptions = fetchOptions;
        loader.adjustmentTransform.copy(upRotationMatrix);
        loader.ellipsoid.copy(this.ellipsoid);
        promise = loader.parse(buffer);
        break;
      }
      case "cmpt": {
        const loader = new CMPTLoader(manager);
        loader.workingPath = workingPath;
        loader.fetchOptions = fetchOptions;
        loader.adjustmentTransform.copy(upRotationMatrix);
        loader.ellipsoid.copy(this.ellipsoid);
        promise = loader.parse(buffer).then((res) => res.scene);
        break;
      }
      // 3DTILES_content_gltf
      case "gltf":
      case "glb": {
        const loader = new GLTFExtensionLoader(manager);
        loader.workingPath = workingPath;
        loader.fetchOptions = fetchOptions;
        promise = loader.parse(buffer);
        break;
      }
      default:
        console.warn(`TilesRenderer: Content type "${fileType}" not supported.`);
        promise = Promise.resolve(null);
        break;
    }
    const stats = this.stats;
    if (this._loadingTiles === false && stats.parsing + stats.downloading > 0) {
      this.dispatchEvent({ type: "tiles-load-start" });
      this._loadingTiles = true;
    }
    const result = await promise;
    let scene;
    let metadata;
    if (result.isObject3D) {
      scene = result;
      metadata = null;
    } else {
      scene = result.scene;
      metadata = result;
    }
    await this.invokeAllPlugins((plugin) => {
      return plugin.processTileModel && plugin.processTileModel(scene, tile);
    });
    if (cached._loadIndex !== loadIndex) {
      return;
    }
    scene.updateMatrix();
    if (fileType === "glb" || fileType === "gltf") {
      scene.matrix.multiply(upRotationMatrix);
    }
    scene.matrix.premultiply(cachedTransform);
    scene.matrix.decompose(scene.position, scene.quaternion, scene.scale);
    scene.traverse((c) => {
      c[INITIAL_FRUSTUM_CULLED] = c.frustumCulled;
    });
    updateFrustumCulled(scene, !this.autoDisableRendererCulling);
    if (REVISION_LESS_1652) {
      scene.traverse((c) => {
        c.raycast = this._overridenRaycast;
      });
    }
    const materials = [];
    const geometry = [];
    const textures = [];
    scene.traverse((c) => {
      if (c.geometry) {
        geometry.push(c.geometry);
      }
      if (c.material) {
        const material = c.material;
        materials.push(c.material);
        for (const key in material) {
          const value = material[key];
          if (value && value.isTexture) {
            textures.push(value);
          }
        }
      }
    });
    cached.materials = materials;
    cached.geometry = geometry;
    cached.textures = textures;
    cached.scene = scene;
    cached.metadata = metadata;
    cached.bytesUsed = estimateBytesUsed2(scene);
    this.dispatchEvent({
      type: "load-model",
      scene,
      tile
    });
    if (this._loadingTiles === true && stats.parsing + stats.downloading === 1) {
      this.dispatchEvent({ type: "tiles-load-end" });
      this._loadingTiles = false;
    }
  }
  disposeTile(tile) {
    super.disposeTile(tile);
    const cached = tile.cached;
    if (cached.scene) {
      const materials = cached.materials;
      const geometry = cached.geometry;
      const textures = cached.textures;
      const parent = cached.scene.parent;
      cached.scene.traverse((child) => {
        if (child.userData.meshFeatures) {
          child.userData.meshFeatures.dispose();
        }
        if (child.userData.structuralMetadata) {
          child.userData.structuralMetadata.dispose();
        }
      });
      for (let i = 0, l = geometry.length; i < l; i++) {
        geometry[i].dispose();
      }
      for (let i = 0, l = materials.length; i < l; i++) {
        materials[i].dispose();
      }
      for (let i = 0, l = textures.length; i < l; i++) {
        const texture = textures[i];
        if (texture.image instanceof ImageBitmap) {
          texture.image.close();
        }
        texture.dispose();
      }
      if (parent) {
        parent.remove(cached.scene);
      }
      this.dispatchEvent({
        type: "dispose-model",
        scene: cached.scene,
        tile
      });
      cached.scene = null;
      cached.materials = null;
      cached.textures = null;
      cached.geometry = null;
      cached.metadata = null;
    }
    cached._loadIndex++;
  }
  setTileVisible(tile, visible) {
    const scene = tile.cached.scene;
    const visibleTiles = this.visibleTiles;
    const group = this.group;
    if (visible) {
      group.add(scene);
      visibleTiles.add(tile);
      scene.updateMatrixWorld(true);
    } else {
      group.remove(scene);
      visibleTiles.delete(tile);
    }
    this.dispatchEvent({
      type: "tile-visibility-change",
      scene,
      tile,
      visible
    });
  }
  setTileActive(tile, active) {
    const activeTiles = this.activeTiles;
    if (active) {
      activeTiles.add(tile);
    } else {
      activeTiles.delete(tile);
    }
  }
  calculateError(tile) {
    const cached = tile.cached;
    const cameras = this.cameras;
    const cameraInfo = this.cameraInfo;
    const boundingVolume = cached.boundingVolume;
    let maxError = -Infinity;
    let minDistance = Infinity;
    for (let i = 0, l = cameras.length; i < l; i++) {
      const info = cameraInfo[i];
      let error;
      if (info.isOrthographic) {
        const pixelSize = info.pixelSize;
        error = tile.geometricError / pixelSize;
      } else {
        const distance = boundingVolume.distanceToPoint(info.position);
        const sseDenominator = info.sseDenominator;
        error = tile.geometricError / (distance * sseDenominator);
        minDistance = Math.min(minDistance, distance);
      }
      maxError = Math.max(maxError, error);
    }
    tile.__distanceFromCamera = minDistance;
    tile.__error = maxError;
  }
  tileInView(tile) {
    const cached = tile.cached;
    const boundingVolume = cached.boundingVolume;
    const cameraInfo = this.cameraInfo;
    for (let i = 0, l = cameraInfo.length; i < l; i++) {
      const frustum = cameraInfo[i].frustum;
      if (boundingVolume.intersectsFrustum(frustum)) {
        return true;
      }
    }
    return false;
  }
  // TODO: deprecate this function and provide a plugin to help with this
  // adjust the rotation of the group such that Y is altitude, X is North, and Z is East
  setLatLonToYUp(lat, lon) {
    const { ellipsoid, group } = this;
    _euler2.set(Math.PI / 2, Math.PI / 2, 0);
    _mat2.makeRotationFromEuler(_euler2);
    ellipsoid.getEastNorthUpFrame(lat, lon, group.matrix).multiply(_mat2).invert().decompose(
      group.position,
      group.quaternion,
      group.scale
    );
    group.updateMatrixWorld(true);
  }
};
[
  ["onLoadTileSet", "load-tile-set"],
  ["onLoadModel", "load-model"],
  ["onDisposeModel", "dispose-model"],
  ["onTileVisibilityChange", "tile-visibility-change"]
].forEach(([methodName, eventName]) => {
  const cachedName = Symbol(methodName);
  Object.defineProperty(
    TilesRenderer.prototype,
    methodName,
    {
      get() {
        return this[cachedName] || null;
      },
      set(cb) {
        console.warn(`TilesRenderer: "${methodName}" has been deprecated in favor of the "${eventName}" event.`);
        if (this[cachedName]) {
          this.removeEventListener(eventName, this[cachedName]);
        }
        this[cachedName] = cb;
        this.addEventListener(eventName, cb);
      }
    }
  );
});

// node_modules/three/examples/jsm/loaders/DRACOLoader.js
import {
  BufferAttribute as BufferAttribute5,
  BufferGeometry as BufferGeometry5,
  Color as Color3,
  FileLoader as FileLoader3,
  Loader as Loader2,
  LinearSRGBColorSpace as LinearSRGBColorSpace2,
  SRGBColorSpace as SRGBColorSpace2
} from "three";
var _taskCache = /* @__PURE__ */ new WeakMap();
var DRACOLoader = class extends Loader2 {
  constructor(manager) {
    super(manager);
    this.decoderPath = "";
    this.decoderConfig = {};
    this.decoderBinary = null;
    this.decoderPending = null;
    this.workerLimit = 4;
    this.workerPool = [];
    this.workerNextTaskID = 1;
    this.workerSourceURL = "";
    this.defaultAttributeIDs = {
      position: "POSITION",
      normal: "NORMAL",
      color: "COLOR",
      uv: "TEX_COORD"
    };
    this.defaultAttributeTypes = {
      position: "Float32Array",
      normal: "Float32Array",
      color: "Float32Array",
      uv: "Float32Array"
    };
  }
  setDecoderPath(path) {
    this.decoderPath = path;
    return this;
  }
  setDecoderConfig(config) {
    this.decoderConfig = config;
    return this;
  }
  setWorkerLimit(workerLimit) {
    this.workerLimit = workerLimit;
    return this;
  }
  load(url, onLoad, onProgress, onError) {
    const loader = new FileLoader3(this.manager);
    loader.setPath(this.path);
    loader.setResponseType("arraybuffer");
    loader.setRequestHeader(this.requestHeader);
    loader.setWithCredentials(this.withCredentials);
    loader.load(url, (buffer) => {
      this.parse(buffer, onLoad, onError);
    }, onProgress, onError);
  }
  parse(buffer, onLoad, onError = () => {
  }) {
    this.decodeDracoFile(buffer, onLoad, null, null, SRGBColorSpace2).catch(onError);
  }
  decodeDracoFile(buffer, callback, attributeIDs, attributeTypes, vertexColorSpace = LinearSRGBColorSpace2, onError = () => {
  }) {
    const taskConfig = {
      attributeIDs: attributeIDs || this.defaultAttributeIDs,
      attributeTypes: attributeTypes || this.defaultAttributeTypes,
      useUniqueIDs: !!attributeIDs,
      vertexColorSpace
    };
    return this.decodeGeometry(buffer, taskConfig).then(callback).catch(onError);
  }
  decodeGeometry(buffer, taskConfig) {
    const taskKey = JSON.stringify(taskConfig);
    if (_taskCache.has(buffer)) {
      const cachedTask = _taskCache.get(buffer);
      if (cachedTask.key === taskKey) {
        return cachedTask.promise;
      } else if (buffer.byteLength === 0) {
        throw new Error(
          "THREE.DRACOLoader: Unable to re-decode a buffer with different settings. Buffer has already been transferred."
        );
      }
    }
    let worker;
    const taskID = this.workerNextTaskID++;
    const taskCost = buffer.byteLength;
    const geometryPending = this._getWorker(taskID, taskCost).then((_worker) => {
      worker = _worker;
      return new Promise((resolve, reject) => {
        worker._callbacks[taskID] = { resolve, reject };
        worker.postMessage({ type: "decode", id: taskID, taskConfig, buffer }, [buffer]);
      });
    }).then((message) => this._createGeometry(message.geometry));
    geometryPending.catch(() => true).then(() => {
      if (worker && taskID) {
        this._releaseTask(worker, taskID);
      }
    });
    _taskCache.set(buffer, {
      key: taskKey,
      promise: geometryPending
    });
    return geometryPending;
  }
  _createGeometry(geometryData) {
    const geometry = new BufferGeometry5();
    if (geometryData.index) {
      geometry.setIndex(new BufferAttribute5(geometryData.index.array, 1));
    }
    for (let i = 0; i < geometryData.attributes.length; i++) {
      const result = geometryData.attributes[i];
      const name2 = result.name;
      const array = result.array;
      const itemSize = result.itemSize;
      const attribute = new BufferAttribute5(array, itemSize);
      if (name2 === "color") {
        this._assignVertexColorSpace(attribute, result.vertexColorSpace);
        attribute.normalized = array instanceof Float32Array === false;
      }
      geometry.setAttribute(name2, attribute);
    }
    return geometry;
  }
  _assignVertexColorSpace(attribute, inputColorSpace) {
    if (inputColorSpace !== SRGBColorSpace2) return;
    const _color = new Color3();
    for (let i = 0, il = attribute.count; i < il; i++) {
      _color.fromBufferAttribute(attribute, i).convertSRGBToLinear();
      attribute.setXYZ(i, _color.r, _color.g, _color.b);
    }
  }
  _loadLibrary(url, responseType) {
    const loader = new FileLoader3(this.manager);
    loader.setPath(this.decoderPath);
    loader.setResponseType(responseType);
    loader.setWithCredentials(this.withCredentials);
    return new Promise((resolve, reject) => {
      loader.load(url, resolve, void 0, reject);
    });
  }
  preload() {
    this._initDecoder();
    return this;
  }
  _initDecoder() {
    if (this.decoderPending) return this.decoderPending;
    const useJS = typeof WebAssembly !== "object" || this.decoderConfig.type === "js";
    const librariesPending = [];
    if (useJS) {
      librariesPending.push(this._loadLibrary("draco_decoder.js", "text"));
    } else {
      librariesPending.push(this._loadLibrary("draco_wasm_wrapper.js", "text"));
      librariesPending.push(this._loadLibrary("draco_decoder.wasm", "arraybuffer"));
    }
    this.decoderPending = Promise.all(librariesPending).then((libraries) => {
      const jsContent = libraries[0];
      if (!useJS) {
        this.decoderConfig.wasmBinary = libraries[1];
      }
      const fn = DRACOWorker.toString();
      const body = [
        "/* draco decoder */",
        jsContent,
        "",
        "/* worker */",
        fn.substring(fn.indexOf("{") + 1, fn.lastIndexOf("}"))
      ].join("\n");
      this.workerSourceURL = URL.createObjectURL(new Blob([body]));
    });
    return this.decoderPending;
  }
  _getWorker(taskID, taskCost) {
    return this._initDecoder().then(() => {
      if (this.workerPool.length < this.workerLimit) {
        const worker2 = new Worker(this.workerSourceURL);
        worker2._callbacks = {};
        worker2._taskCosts = {};
        worker2._taskLoad = 0;
        worker2.postMessage({ type: "init", decoderConfig: this.decoderConfig });
        worker2.onmessage = function(e) {
          const message = e.data;
          switch (message.type) {
            case "decode":
              worker2._callbacks[message.id].resolve(message);
              break;
            case "error":
              worker2._callbacks[message.id].reject(message);
              break;
            default:
              console.error('THREE.DRACOLoader: Unexpected message, "' + message.type + '"');
          }
        };
        this.workerPool.push(worker2);
      } else {
        this.workerPool.sort(function(a, b) {
          return a._taskLoad > b._taskLoad ? -1 : 1;
        });
      }
      const worker = this.workerPool[this.workerPool.length - 1];
      worker._taskCosts[taskID] = taskCost;
      worker._taskLoad += taskCost;
      return worker;
    });
  }
  _releaseTask(worker, taskID) {
    worker._taskLoad -= worker._taskCosts[taskID];
    delete worker._callbacks[taskID];
    delete worker._taskCosts[taskID];
  }
  debug() {
    console.log("Task load: ", this.workerPool.map((worker) => worker._taskLoad));
  }
  dispose() {
    for (let i = 0; i < this.workerPool.length; ++i) {
      this.workerPool[i].terminate();
    }
    this.workerPool.length = 0;
    if (this.workerSourceURL !== "") {
      URL.revokeObjectURL(this.workerSourceURL);
    }
    return this;
  }
};
function DRACOWorker() {
  let decoderConfig;
  let decoderPending;
  onmessage = function(e) {
    const message = e.data;
    switch (message.type) {
      case "init":
        decoderConfig = message.decoderConfig;
        decoderPending = new Promise(function(resolve) {
          decoderConfig.onModuleLoaded = function(draco) {
            resolve({ draco });
          };
          DracoDecoderModule(decoderConfig);
        });
        break;
      case "decode":
        const buffer = message.buffer;
        const taskConfig = message.taskConfig;
        decoderPending.then((module) => {
          const draco = module.draco;
          const decoder = new draco.Decoder();
          try {
            const geometry = decodeGeometry(draco, decoder, new Int8Array(buffer), taskConfig);
            const buffers = geometry.attributes.map((attr) => attr.array.buffer);
            if (geometry.index) buffers.push(geometry.index.array.buffer);
            self.postMessage({ type: "decode", id: message.id, geometry }, buffers);
          } catch (error) {
            console.error(error);
            self.postMessage({ type: "error", id: message.id, error: error.message });
          } finally {
            draco.destroy(decoder);
          }
        });
        break;
    }
  };
  function decodeGeometry(draco, decoder, array, taskConfig) {
    const attributeIDs = taskConfig.attributeIDs;
    const attributeTypes = taskConfig.attributeTypes;
    let dracoGeometry;
    let decodingStatus;
    const geometryType = decoder.GetEncodedGeometryType(array);
    if (geometryType === draco.TRIANGULAR_MESH) {
      dracoGeometry = new draco.Mesh();
      decodingStatus = decoder.DecodeArrayToMesh(array, array.byteLength, dracoGeometry);
    } else if (geometryType === draco.POINT_CLOUD) {
      dracoGeometry = new draco.PointCloud();
      decodingStatus = decoder.DecodeArrayToPointCloud(array, array.byteLength, dracoGeometry);
    } else {
      throw new Error("THREE.DRACOLoader: Unexpected geometry type.");
    }
    if (!decodingStatus.ok() || dracoGeometry.ptr === 0) {
      throw new Error("THREE.DRACOLoader: Decoding failed: " + decodingStatus.error_msg());
    }
    const geometry = { index: null, attributes: [] };
    for (const attributeName in attributeIDs) {
      const attributeType = self[attributeTypes[attributeName]];
      let attribute;
      let attributeID;
      if (taskConfig.useUniqueIDs) {
        attributeID = attributeIDs[attributeName];
        attribute = decoder.GetAttributeByUniqueId(dracoGeometry, attributeID);
      } else {
        attributeID = decoder.GetAttributeId(dracoGeometry, draco[attributeIDs[attributeName]]);
        if (attributeID === -1) continue;
        attribute = decoder.GetAttribute(dracoGeometry, attributeID);
      }
      const attributeResult = decodeAttribute(draco, decoder, dracoGeometry, attributeName, attributeType, attribute);
      if (attributeName === "color") {
        attributeResult.vertexColorSpace = taskConfig.vertexColorSpace;
      }
      geometry.attributes.push(attributeResult);
    }
    if (geometryType === draco.TRIANGULAR_MESH) {
      geometry.index = decodeIndex(draco, decoder, dracoGeometry);
    }
    draco.destroy(dracoGeometry);
    return geometry;
  }
  function decodeIndex(draco, decoder, dracoGeometry) {
    const numFaces = dracoGeometry.num_faces();
    const numIndices = numFaces * 3;
    const byteLength = numIndices * 4;
    const ptr = draco._malloc(byteLength);
    decoder.GetTrianglesUInt32Array(dracoGeometry, byteLength, ptr);
    const index = new Uint32Array(draco.HEAPF32.buffer, ptr, numIndices).slice();
    draco._free(ptr);
    return { array: index, itemSize: 1 };
  }
  function decodeAttribute(draco, decoder, dracoGeometry, attributeName, attributeType, attribute) {
    const numComponents = attribute.num_components();
    const numPoints = dracoGeometry.num_points();
    const numValues = numPoints * numComponents;
    const byteLength = numValues * attributeType.BYTES_PER_ELEMENT;
    const dataType = getDracoDataType(draco, attributeType);
    const ptr = draco._malloc(byteLength);
    decoder.GetAttributeDataArrayForAllPoints(dracoGeometry, attribute, dataType, byteLength, ptr);
    const array = new attributeType(draco.HEAPF32.buffer, ptr, numValues).slice();
    draco._free(ptr);
    return {
      name: attributeName,
      array,
      itemSize: numComponents
    };
  }
  function getDracoDataType(draco, attributeType) {
    switch (attributeType) {
      case Float32Array:
        return draco.DT_FLOAT32;
      case Int8Array:
        return draco.DT_INT8;
      case Int16Array:
        return draco.DT_INT16;
      case Int32Array:
        return draco.DT_INT32;
      case Uint8Array:
        return draco.DT_UINT8;
      case Uint16Array:
        return draco.DT_UINT16;
      case Uint32Array:
        return draco.DT_UINT32;
    }
  }
}
export {
  CesiumIonAuthPlugin,
  DRACOLoader,
  Ellipsoid,
  GLTFExtensionsPlugin,
  GoogleCloudAuthPlugin,
  TileCompressionPlugin,
  TilesFadePlugin,
  TilesRenderer,
  WGS84_ELLIPSOID
};
