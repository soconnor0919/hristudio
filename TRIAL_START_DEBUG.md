# Trial Start Debug Guide

## ❌ **Problem**: "I can't start the trial"

This guide will help you systematically debug why the trial start functionality isn't working.

---

## 🔍 **Step 1: Verify System Setup**

### Database Connection
```bash
# Check if database is running
docker ps | grep postgres

# If not running, start it
bun run docker:up

# Check database schema is up to date
bun db:push

# Verify seed data exists
bun db:seed
```

### Build Status
```bash
# Ensure project builds without errors
bun run build

# Should complete successfully with no TypeScript errors
```

---

## 🔍 **Step 2: Browser-Based Testing**

### Access the Wizard Interface
1. Start dev server: `bun dev`
2. Open browser: `http://localhost:3000`
3. Login: `sean@soconnor.dev` / `password123`
4. Navigate: Studies → [First Study] → Trials → [First Trial] → Wizard Interface

### Check Browser Console
Open Developer Tools (F12) and look for:

**Expected Debug Messages** (when clicking "Start Trial"):
```
[WizardInterface] Starting trial: <id> Current status: scheduled
[WizardControlPanel] Start Trial clicked
```

**Error Messages to Look For**:
- Network errors (red entries in Console)
- tRPC errors (search for "trpc" or "TRPC")
- Authentication errors (401/403 status codes)
- Database errors (check Network tab)

---

## 🔍 **Step 3: Test Database Access**

### Quick API Test
Visit this URL in your browser while dev server is running:
```
http://localhost:3000/api/test-trial
```

**Expected Response**:
```json
{
  "success": true,
  "message": "Database connection working",
  "trials": [...],
  "count": 4
}
```

**If you get an error**, the database connection is broken.

### Check Specific Trial
If the above works, test with a specific trial ID:
```
http://localhost:3000/api/test-trial?id=<trial-id-from-above-response>
```

---

## 🔍 **Step 4: Verify Trial Status**

### Requirements for Starting Trial
1. **Trial must exist** - Check API response has trials
2. **Trial must be "scheduled"** - Status should be "scheduled", not "in_progress" or "completed"  
3. **User must have permissions** - Must be administrator, researcher, or wizard role
4. **Experiment must have steps** - Trial needs an experiment with defined steps

### Check Trial Data
In browser console, after navigating to wizard interface:
```javascript
// Check trial data
console.log("Current trial:", window.location.pathname);

// Check user session
fetch('/api/auth/session').then(r => r.json()).then(console.log);
```

---

## 🔍 **Step 5: tRPC API Testing**

### Test tRPC Endpoint Directly
In browser console on the wizard page:
```javascript
// This should work if you're on the wizard interface page
// Replace 'TRIAL_ID' with actual trial ID from URL
fetch('/api/trpc/trials.get?batch=1&input={"0":{"json":{"id":"TRIAL_ID"}}}')
  .then(r => r.json())
  .then(console.log);
```

### Test Start Trial Endpoint
```javascript
// Test the start trial mutation
fetch('/api/trpc/trials.start', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    "0": {
      "json": {
        "id": "TRIAL_ID_HERE"
      }
    }
  })
}).then(r => r.json()).then(console.log);
```

---

## 🔍 **Step 6: Common Issues & Fixes**

### Issue: "Start Trial" Button Doesn't Respond
- Check browser console for JavaScript errors
- Verify the button isn't disabled
- Check if `isStarting` state is stuck on `true`

### Issue: Network Error / API Not Found
- Check middleware isn't blocking tRPC routes
- Verify NextAuth session is valid
- Check if API routes are properly built

### Issue: Permission Denied
- Check user role: must be administrator, researcher, or wizard
- Verify study membership if role-based access is enabled
- Check `checkTrialAccess` function in trials router

### Issue: "Trial can only be started from scheduled status"
- Current trial status is not "scheduled"
- Find a trial with "scheduled" status or create one manually
- Check seed data created scheduled trials properly

### Issue: Database Connection Error
- Database container not running
- Environment variables missing/incorrect
- Schema not pushed or out of date

---

## 🔧 **Manual Debugging Steps**

### Create Test Trial
If no scheduled trials exist:
```sql
-- Connect to database and create a test trial
INSERT INTO trial (
  id, 
  experiment_id, 
  participant_id, 
  status, 
  session_number,
  scheduled_at,
  created_at,
  updated_at
) VALUES (
  gen_random_uuid(),
  'EXPERIMENT_ID_HERE',
  'PARTICIPANT_ID_HERE', 
  'scheduled',
  1,
  NOW(),
  NOW(),
  NOW()
);
```

### Check User Permissions
```sql
-- Check user system roles
SELECT u.email, usr.role 
FROM users u 
LEFT JOIN user_system_roles usr ON u.id = usr.user_id 
WHERE u.email = 'sean@soconnor.dev';

-- Check study memberships
SELECT u.email, sm.role, s.name as study_name
FROM users u 
LEFT JOIN study_members sm ON u.id = sm.user_id
LEFT JOIN studies s ON sm.study_id = s.id
WHERE u.email = 'sean@soconnor.dev';
```

---

## 🚨 **Emergency Fixes**

### Quick Reset
```bash
# Complete reset of database and seed data
bun run docker:down
bun run docker:up
bun db:push
bun db:seed
```

### Bypass Authentication (Development Only)
In `src/server/api/routers/trials.ts`, temporarily comment out the permission check:
```typescript
// await checkTrialAccess(db, userId, input.id, [
//   "owner",
//   "researcher", 
//   "wizard",
// ]);
```

---

## 📞 **Getting Help**

If none of the above steps resolve the issue:

1. **Provide the following information**:
   - Output of `/api/test-trial` 
   - Browser console errors (screenshots)
   - Network tab showing failed requests
   - Current user session info
   - Trial ID you're trying to start

2. **Include environment details**:
   - Operating system
   - Node.js version (`node --version`)
   - Bun version (`bun --version`)
   - Docker status (`docker ps`)

3. **Steps you've already tried** from this guide

---

## ✅ **Success Indicators**

When trial start is working correctly, you should see:

1. **Debug logs in console**:
   ```
   [WizardInterface] Starting trial: abc123 Current status: scheduled
   [WizardControlPanel] Start Trial clicked
   [WizardInterface] Trial started successfully
   ```

2. **UI changes**:
   - "Start Trial" button disappears/disables
   - Toast notification: "Trial started successfully"
   - Trial status badge changes to "in progress"
   - Control buttons appear (Pause, Next, Complete, Abort)

3. **Database changes**:
   - Trial status changes from "scheduled" to "in_progress"
   - `started_at` timestamp is set
   - Trial event is logged with type "trial_started"

The trial start functionality is working when all three indicators occur successfully.