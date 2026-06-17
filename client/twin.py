"""Python client for the UKy campus digital-twin server (tools/twin_server.py).

The twin runs on its own server; your script talks to it over a small REST API. The
world is SHARED — `twin.agents()` returns every agent currently in the world, including
ones spawned by other people's scripts or from a browser, and any agent you spawn is
immediately visible to them too.

    from twin import Twin

    twin = Twin("http://twin-server.example:8000")   # or just Twin() for localhost
    drone = twin.spawn("drone", position=[0, None, 0], owner="alice")
    drone.set_controls(move=[5, 1, 0])               # fly +X and climb
    print(drone.state()["position"], drone.collisions())
    for other in twin.agents():                      # everyone in the shared world
        print(other["owner"], other["type"], other["position"])
    drone.stop(); drone.despawn()

Pure stdlib (urllib) — no dependencies.
"""
import json
import urllib.error
import urllib.request

__all__ = ["Twin", "Agent", "TwinError"]


class TwinError(RuntimeError):
    pass


class Twin:
    def __init__(self, base_url="http://localhost:8000", owner=None, timeout=10):
        self.base = base_url.rstrip("/")
        self.owner = owner
        self.timeout = timeout

    # ---- low-level REST ----
    def _req(self, method, path, body=None):
        url = self.base + path
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(url, data=data, method=method,
                                     headers={"Content-Type": "application/json",
                                              "User-Agent": "twin-client"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                raw = r.read()
                return json.loads(raw) if raw else {}
        except urllib.error.HTTPError as e:
            try:
                msg = json.loads(e.read()).get("error", str(e))
            except Exception:
                msg = str(e)
            raise TwinError(f"{method} {path}: {msg}") from None
        except urllib.error.URLError as e:
            raise TwinError(f"cannot reach twin server at {self.base} ({e.reason}). "
                            f"Is `python -m tools.twin_server` running?") from None

    # ---- world ----
    def meta(self):
        return self._req("GET", "/api/world/meta")

    def state(self):
        """Full world snapshot: {t, frame, agents:[...]}."""
        return self._req("GET", "/api/world/state")

    def agents(self):
        """Every agent in the shared world (yours and everyone else's)."""
        return self.state().get("agents", [])

    def nearest_building(self, x, z):
        """Nearest building to a scene point: {x,y,z,name,dist} or None."""
        b = self._req("GET", f"/api/world/nearest_building?x={x}&z={z}")
        return b or None

    def spawn(self, type="car", position=None, heading=0, color=None, name=None, owner=None):
        body = {"type": type, "heading": heading}
        if position is not None:
            body["position"] = list(position)
        if color is not None:
            body["color"] = color
        if name is not None:
            body["name"] = name
        body["owner"] = owner if owner is not None else self.owner
        st = self._req("POST", "/api/world/spawn", body)
        if st.get("error"):
            raise TwinError(st["error"])
        return Agent(self, st["id"], st)

    def get(self, agent_id):
        return Agent(self, int(agent_id))

    def despawn(self, agent_id):
        return self._req("DELETE", f"/api/world/agents/{int(agent_id)}").get("ok", False)


class Agent:
    """Handle to one agent in the shared world."""

    def __init__(self, twin, agent_id, initial=None):
        self.twin = twin
        self.id = int(agent_id)
        self._last = initial or {}

    # ---- sensors ----
    def state(self):
        """Full sensor state: position, heading, velocity, surface, collisions, utm, ..."""
        self._last = self.twin._req("GET", f"/api/world/agents/{self.id}")
        return self._last

    def position(self):
        return self.state()["position"]

    def collisions(self):
        return self.state().get("collisions", [])

    # ---- control ----
    def set_controls(self, **controls):
        """Ground: throttle, brake, steer, reverse, handbrake (or left/right for the robot).
        Drone: move=[vx,vy,vz] (world velocity) OR thrust, climb, yawRate."""
        self.twin._req("POST", f"/api/world/agents/{self.id}/controls", controls)
        return self

    def drive_to(self, x, z, y=None, speed=None, arrive_radius=None, stop=True):
        body = {"x": x, "z": z, "stop": stop}
        if y is not None:
            body["y"] = y
        if speed is not None:
            body["speed"] = speed
        if arrive_radius is not None:
            body["arriveRadius"] = arrive_radius
        self.twin._req("POST", f"/api/world/agents/{self.id}/driveTo", body)
        return self

    def stop(self):
        self.twin._req("POST", f"/api/world/agents/{self.id}/stop", {})
        return self

    def despawn(self):
        return self.twin.despawn(self.id)
