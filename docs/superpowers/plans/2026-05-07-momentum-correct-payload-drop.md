# Momentum-Correct Payload Drop Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the payload-drop physics so that releasing a payload mid-flight conserves linear and angular momentum, instead of silently teleporting mass out of the inertia matrix.

**Architecture:** Add a `dropPayload` method to `HexacopterPlant` that computes the momentum-correct velocity adjustment at the instant of release, then update `SimRunner.runAdaptiveLoop` to call it instead of the current bare `updateParameters`. No new files — this is a surgical fix to two existing classes plus their tests.

---

## The Physics Bug

**Current behavior (lines 452-454 of `SimRunner.m`):**
```matlab
if ~dropped && obj.tCurrent >= dropTime
    obj.plant.updateParameters(m_base, cog_base, I_base);
    dropped = true;
end
```

This changes mass/inertia instantly but leaves the body velocity `V` unchanged. That violates conservation of momentum.

**What should happen:**

At the instant of release, the composite body (vehicle + payload) has generalized momentum:
```
p = I6_composite * V_before
```

After release, the vehicle alone must carry its share of that momentum:
```
I6_base * V_after = p
V_after = I6_base \ (I6_composite * V_before)
```

This is the standard momentum-conservation constraint for instantaneous mass separation of a rigid body where the released mass has zero relative velocity at the separation point.

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/+vt/+plant/HexacopterPlant.m` | Modify | Add `dropPayload` method |
| `src/+vt/+sim/SimRunner.m:452-454` | Modify | Call `dropPayload` instead of `updateParameters` |
| `tests/TestHexacopterPlant.m` | Modify | Add momentum-conservation tests |

---

## Task 1: Add momentum-conservation tests to `TestHexacopterPlant`

**Files:**
- Modify: `tests/TestHexacopterPlant.m`

- [ ] **Step 1: Write test — `dropPayload` conserves linear momentum in pure translation**

A vehicle + payload moving in pure translation (no rotation). After drop, the velocity must preserve `m_composite * v = m_base * v_after`, so `v_after = (m_composite / m_base) * v`.

Add to `tests/TestHexacopterPlant.m`:

```matlab
function testDropPayloadConservesLinearMomentum(testCase)
    cfg = vt.config.Config();
    cfg.vehicle.g = 0;
    cfg.sim.groundEnable = false;
    plant = vt.plant.HexacopterPlant(cfg);

    m_base = cfg.vehicle.m;
    I_base = cfg.vehicle.I_params;
    cog_base = cfg.vehicle.CoG(:);

    m_payload = 0.5;
    cog_payload = [0; 0; -0.1];
    [m_comp, I_comp, cog_comp] = vt.utils.addPayload(m_base, I_base, cog_base, m_payload, cog_payload);
    plant.updateParameters(m_comp, cog_comp, I_comp);

    H0 = eye(4); H0(3,4) = 5;
    V0 = [0; 0; 0; 1.0; 0; 0];  % pure x-translation
    plant.reset(H0, V0);

    I6_before = plant.I6;
    p_before = I6_before * V0;

    plant.dropPayload(m_base, cog_base, I_base);

    [~, V_after] = plant.getState();
    I6_after = plant.I6;
    p_after = I6_after * V_after;

    testCase.verifyEqual(p_after, p_before, 'AbsTol', 1e-12);
end
```

- [ ] **Step 2: Write test — `dropPayload` conserves angular momentum with rotation**

A vehicle with both angular and linear velocity. The full 6D generalized momentum must be preserved.

Add to `tests/TestHexacopterPlant.m`:

```matlab
function testDropPayloadConservesGeneralizedMomentum(testCase)
    cfg = vt.config.Config();
    cfg.vehicle.g = 0;
    cfg.sim.groundEnable = false;
    plant = vt.plant.HexacopterPlant(cfg);

    m_base = cfg.vehicle.m;
    I_base = cfg.vehicle.I_params;
    cog_base = cfg.vehicle.CoG(:);

    m_payload = 0.5;
    cog_payload = [0.05; -0.03; -0.1];
    [m_comp, I_comp, cog_comp] = vt.utils.addPayload(m_base, I_base, cog_base, m_payload, cog_payload);
    plant.updateParameters(m_comp, cog_comp, I_comp);

    H0 = eye(4); H0(3,4) = 5;
    V0 = [0.1; -0.2; 0.15; 0.8; -0.5; 0.3];
    plant.reset(H0, V0);

    I6_before = plant.I6;
    p_before = I6_before * V0;

    plant.dropPayload(m_base, cog_base, I_base);

    [~, V_after] = plant.getState();
    I6_after = plant.I6;
    p_after = I6_after * V_after;

    testCase.verifyEqual(p_after, p_before, 'AbsTol', 1e-12);
end
```

- [ ] **Step 3: Write test — `dropPayload` with zero payload mass is a no-op**

Edge case: dropping nothing should not change velocity.

Add to `tests/TestHexacopterPlant.m`:

```matlab
function testDropPayloadNoOpWhenMassUnchanged(testCase)
    cfg = vt.config.Config();
    cfg.vehicle.g = 0;
    cfg.sim.groundEnable = false;
    plant = vt.plant.HexacopterPlant(cfg);

    H0 = eye(4); H0(3,4) = 5;
    V0 = [0.1; -0.2; 0.15; 0.8; -0.5; 0.3];
    plant.reset(H0, V0);

    m_same = cfg.vehicle.m;
    I_same = cfg.vehicle.I_params;
    cog_same = cfg.vehicle.CoG(:);

    plant.dropPayload(m_same, cog_same, I_same);

    [~, V_after] = plant.getState();
    testCase.verifyEqual(V_after, V0, 'AbsTol', 1e-12);
end
```

- [ ] **Step 4: Run the tests to verify they fail**

```matlab
runtests('tests/TestHexacopterPlant.m', 'ProcedureName', 'testDropPayload*')
```

Expected: FAIL — `dropPayload` method does not exist yet.

- [ ] **Step 5: Commit**

```bash
git add tests/TestHexacopterPlant.m
git commit -m "test(plant): add momentum-conservation tests for payload drop"
```

---

## Task 2: Implement `dropPayload` on `HexacopterPlant`

**Files:**
- Modify: `src/+vt/+plant/HexacopterPlant.m:111-126` (add method after `updateParameters`)

- [ ] **Step 1: Add `dropPayload` method**

Add to the public methods section of `HexacopterPlant`, after `updateParameters`:

```matlab
function dropPayload(obj, m_new, CoG_new, Iparams_new)
    %DROPPAYLOAD Release payload with momentum conservation.
    %   Adjusts body velocity so that generalized momentum is
    %   preserved across the instantaneous mass/inertia change.
    %
    %   Physics:
    %     p = I6_before * V_before  (momentum before drop)
    %     V_after = I6_after \ p    (solve for new velocity)
    %
    %   Inputs:
    %     m_new - post-drop mass [kg].
    %     CoG_new - 3x1 post-drop center of gravity [m].
    %     Iparams_new - 1x6 post-drop inertia parameters.
    p = obj.I6 * obj.V;
    obj.updateParameters(m_new, CoG_new, Iparams_new);
    obj.V = obj.I6 \ p;
end
```

- [ ] **Step 2: Run tests to verify they pass**

```matlab
runtests('tests/TestHexacopterPlant.m')
```

Expected: all tests PASS, including the three new momentum-conservation tests.

- [ ] **Step 3: Commit**

```bash
git add src/+vt/+plant/HexacopterPlant.m
git commit -m "feat(plant): add momentum-conserving dropPayload method"
```

---

## Task 3: Wire `SimRunner` to use `dropPayload`

**Files:**
- Modify: `src/+vt/+sim/SimRunner.m:443-459`

- [ ] **Step 1: Update `runAdaptiveLoop` to call `dropPayload`**

Replace the current drop handling:

```matlab
if ~dropped && obj.tCurrent >= dropTime
    obj.plant.updateParameters(m_base, cog_base, I_base);
    dropped = true;
end
```

With:

```matlab
if ~dropped && obj.tCurrent >= dropTime
    obj.plant.dropPayload(m_base, cog_base, I_base);
    dropped = true;
end
```

This is a one-line change: `updateParameters` → `dropPayload`.

- [ ] **Step 2: Run all tests**

```matlab
runtests('tests')
```

Expected: all tests PASS.

- [ ] **Step 3: Commit**

```bash
git add src/+vt/+sim/SimRunner.m
git commit -m "fix(sim): use momentum-conserving drop in adaptive loop"
```
