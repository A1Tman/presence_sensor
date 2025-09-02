# Security Setup Guide

## 1. Initial Project Setup

### Create Required Files:
```bash
# In your project root directory
touch .gitignore
mkdir -p config
touch config/secrets.h.template
# DO NOT create config/secrets.h yet - we'll copy from template
```

### Copy the Files I Provided:
1. **Copy `.gitignore`** from the artifact into your project root
2. **Copy `secrets.h.template`** into `config/`  
3. **Copy the secure `main.c`** into your `main/` folder (replace existing)

## 2. Configure Your Secrets

### Create Your Secrets File:
```bash
# In project root directory
cp config/secrets.h.template config/secrets.h
```

### Edit `config/secrets.h` with YOUR actual values:
```c
// Update these with YOUR network settings:
#define WIFI_SSID      "YourActualWiFiName"
#define WIFI_PASSWORD  "YourActualWiFiPassword"

// Update with YOUR Home Assistant IP:
#define MQTT_BROKER_HOST    "192.168.1.XXX"  // Your HA IP

// Make each device unique:
#define DEVICE_ID          "presence_sensor_living_room"  
#define DEVICE_NAME        "Living Room Presence"
#define DEVICE_LOCATION    "Living Room"
```

## 3. Verify Security

### Check `.gitignore` is Working:
```bash
git status
# Should NOT show config/secrets.h in untracked files
# Should NOT show build/ folder
```

### Verify Secrets Loading:
- Build and flash the firmware
- Check serial monitor for "Configuration validation passed ✓"
- Should show your WiFi connecting successfully

## 4. Git Repository Setup

### Before First Commit:
```bash
# Initialize repo
git init

# Verify secrets are ignored
git status
# secrets.h should NOT appear in the list

# Add safe files only
git add .gitignore
git add config/secrets.h.template  
git add main/
git add CMakeLists.txt
git add README.md

# First commit (no secrets!)
git commit -m "Initial secure ESP-IDF presence sensor project"
```

## 5. Security Features Implemented

✅ **Secrets Management**
- All credentials in `secrets.h` (git-ignored)
- Template file for easy setup
- Compile-time validation
- No hardcoded credentials in code

✅ **Secure WiFi**
- WPA2-PSK minimum security
- Power management configuration
- Proper authentication modes
- Connection retry logic

✅ **Build Security**
- Build artifacts excluded from git
- No sensitive information in repository
- Clean separation of code and configuration

✅ **Runtime Security**
- Configuration validation at startup
- Secure logging (credentials masked)
- Memory monitoring
- Error handling

## 6. What Gets Committed vs. Ignored

### ✅ Safe to Commit:
- `main.c` (no secrets)
- `CMakeLists.txt`
- `secrets.h.template` (template only)
- `.gitignore`
- Documentation

### ❌ NEVER Commit:
- `secrets.h` (contains actual credentials)
- `build/` folder (contains compiled binaries)
- `sdkconfig` (may contain sensitive settings)
- Any `.key`, `.crt`, `.pem` files

## 7. Adding Team Members

When someone else needs to work on the project:

1. They clone the repository
2. They copy `secrets.h.template` to `secrets.h`
3. They configure their own credentials in `secrets.h`
4. They can build and run without any secrets in the repo

## 8. Production Deployment

For production devices:
- Use unique `DEVICE_ID` for each physical device
- Consider using device certificates for MQTT
- Enable OTA signature verification
- Use TLS for MQTT in production networks

## ⚠️ Security Checklist

Before any commit:
- [ ] `secrets.h` is in `.gitignore`
- [ ] No credentials in any committed files
- [ ] `git status` shows no sensitive files
- [ ] Template file has placeholder values only
- [ ] Build folder is ignored

## 🆘 If You Accidentally Commit Secrets

If you accidentally commit secrets:
```bash
# Remove from staging
git reset HEAD secrets.h

# Remove from history (if already committed)
git filter-branch --force --index-filter \
'git rm --cached --ignore-unmatch secrets.h' \
--prune-empty --tag-name-filter cat -- --all

# Force push to overwrite remote
git push origin --force --all
```

Then immediately change all exposed credentials (WiFi passwords, MQTT passwords, etc.).
