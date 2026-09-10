### Results from a call to /AuditLog corresponding to a sequence of Vent, Open, Close, Pump, Chamber Clean
- 99476: last action before starting sequence
- 77-80: Vent
- 80: Open (no output in audit)
- 81: Close
- 82: Pump
- 83-84: Recipe Start and User Logout

> Since user logged outbefore recipe was done, it seems recipe end was not tracked?

### Output

```bash
{
    "time": "2026-09-10T02:47:06.508Z",
    "category": "I",
    "id": 94784,
    "module": null,
    "subsystem": null,
    "device": null,
    "code": null,
    "shortDescription": null,
    "description": "User 'resl-mm' logged out",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T02:17:06.105Z",
    "category": "M",
    "id": 94783,
    "module": "RIE #1",
    "subsystem": null,
    "device": null,
    "code": null,
    "shortDescription": null,
    "description": "Recipe 'Start' button pressed",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T02:13:41.193Z",
    "category": "I",
    "id": 94782,
    "module": "RIE #1",
    "subsystem": null,
    "device": "Chamber Pump",
    "code": null,
    "shortDescription": null,
    "description": "User changed value of setting 'Default Auto Zero' to Yes",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T02:13:16.21Z",
    "category": "I",
    "id": 94781,
    "module": "PMC1",
    "subsystem": null,
    "device": "PMC1",
    "code": null,
    "shortDescription": null,
    "description": "Lid closed. Asking User if chamber contains a substrate.",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T02:11:48.985Z",
    "category": "I",
    "id": 94780,
    "module": null,
    "subsystem": null,
    "device": "PMC1",
    "code": null,
    "shortDescription": null,
    "description": "Requesting user attention",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T02:05:48.653Z",
    "category": "I",
    "id": 94779,
    "module": "RIE #1",
    "subsystem": null,
    "device": "Chamber Vent",
    "code": null,
    "shortDescription": null,
    "description": "User changed value of setting 'Turbo state when venting' to On",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T02:05:48.635Z",
    "category": "I",
    "id": 94778,
    "module": "RIE #1",
    "subsystem": null,
    "device": "Chamber Vent",
    "code": null,
    "shortDescription": null,
    "description": "User changed value of setting 'Soft Pump in Purge' to Use if fitted",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T02:05:48.627Z",
    "category": "I",
    "id": 94777,
    "module": "RIE #1",
    "subsystem": null,
    "device": "Chamber Vent",
    "code": null,
    "shortDescription": null,
    "description": "User changed value of setting 'Soft Vent in Purge' to Use if fitted",
    "user": "resl-mm"
  },
  {
    "time": "2026-09-10T01:52:15.355Z",
    "category": "I",
    "id": 94776,
    "module": "RIE #1",
    "subsystem": null,
    "device": "Chamber Pump",
    "code": null,
    "shortDescription": null,
    "description": "User changed value of setting 'Default Auto Zero' to Yes",
    "user": "resl-mm"
  }
```
