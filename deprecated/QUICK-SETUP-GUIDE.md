# 📦 Drop-In Setup Instructions

## 🎯 **Simple 3-Step Setup**

### **Step 1: Copy Build Scripts**
Copy these files to your **project root** (same level as `firmware/` folder):

```
📁 drone_main/                    ← Your project root
├── 🔧 build_all.bat             ← Master build menu [97]
├── 🔧 build_host.bat            ← Host testing build [91]  
├── 🔧 build_lpc4330.bat         ← LPC4330 target build [92]
├── 🔧 setup_environment.bat     ← Environment checker [93]
├── 🔧 install_requirements.bat  ← Auto-installer [98]
├── 🔧 program_lpc4330.bat       ← Programming script [94]
├── 🔧 build_all.sh              ← Linux/macOS build [96]
├── 🖥️ firmware_uploader.py       ← GUI upload tool [95]
├── 📖 BUILD-SYSTEM-README.md     ← Complete documentation [99]
└── firmware/                    ← Your existing firmware folder
```

### **Step 2: Copy Core Files**
Copy these missing files to `firmware/Core/`:

```
📁 firmware/Core/
├── 🔧 startup.c          ← [81]
├── 🔧 system_lpc43xx.c   ← [82]  
├── 🔧 rtos_hooks.c       ← [83]
├── 🔧 dualcore.c         ← [84]
├── 🔧 interrupts.c       ← [85]
└── 🔧 syscalls.c         ← [86]
```

### **Step 3: Copy Enhanced Files**
Replace/add these files from the safety system:

```
📁 firmware/Config/
├── 🔄 config.h           ← Replace with [70]
└── 🔄 config.c           ← Replace with [69]

📁 firmware/App/
└── 🔄 init.c             ← Replace with [53]

📁 firmware/Middleware/
├── ➕ safety_system.h     ← Add [72]  
├── ➕ safety_system.c     ← Add [71]
├── ➕ spi_interface.h     ← Add [88]
└── ➕ spi_interface.c     ← Add [87]

📁 firmware/
└── 🔄 CMakeLists.txt      ← Replace with [79]

📁 firmware/Core/
└── 🔄 CMakeLists.txt      ← Replace with [80]
```

---

## 🚀 **Start Building**

### **Windows:**
```batch
# Run the master build system
build_all.bat

# Or individual steps:
setup_environment.bat    # Check tools
build_host.bat          # Test build
build_lpc4330.bat       # Target build  
python firmware_uploader.py  # Upload GUI
```

### **Linux/macOS:**
```bash
# Make executable
chmod +x build_all.sh

# Run complete build
./build_all.sh
```

---

## ✅ **File Reference**

| File | Purpose | ID |
|------|---------|----| 
| build_all.bat | Windows master menu | [97] |
| build_host.bat | Windows host build | [91] |
| build_lpc4330.bat | Windows target build | [92] |
| setup_environment.bat | Environment check | [93] |
| install_requirements.bat | Auto-installer | [98] |
| program_lpc4330.bat | Programming script | [94] |
| build_all.sh | Linux/macOS build | [96] |
| firmware_uploader.py | GUI upload tool | [95] |
| BUILD-SYSTEM-README.md | Documentation | [99] |

---

## 🎉 **That's It!**

Your complete build and programming system is ready:
- ✅ **Fixed CMake builds** (no more missing files)
- ✅ **Professional safety system** with crash detection  
- ✅ **SPI interface** for external modules
- ✅ **GUI programming tool** with progress tracking
- ✅ **Cross-platform support** (Windows + Linux/macOS)
- ✅ **Auto-installation** of required tools

**Just run `build_all.bat` and choose what you want to do!** 🚁✨