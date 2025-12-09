# 🚁 Drone Build & Programming Tools

## 🎯 **Complete Drop-In Build System**

This is a comprehensive build and programming system for your LPC4330 drone firmware. Everything is designed to be **drop-in** - just copy the files and start building!

---

## 📁 **Files Included**

### **🔧 Build Scripts (Windows)**
- **`build_all.bat`** - Master build menu with all options
- **`build_host.bat`** - Build for PC testing (MinGW)
- **`build_lpc4330.bat`** - Build for LPC4330 target (ARM GCC)
- **`setup_environment.bat`** - Check installed tools
- **`install_requirements.bat`** - Auto-install development tools
- **`program_lpc4330.bat`** - Program via OpenOCD/JTAG

### **🔧 Build Scripts (Linux/macOS)**
- **`build_all.sh`** - Complete build script for Unix systems
- Make executable with: `chmod +x *.sh`

### **🖥️ GUI Tools**
- **`firmware_uploader.py`** - Professional firmware upload GUI
  - Multiple programmer support (OpenOCD, DFU, Serial)
  - Progress tracking and logging
  - Automatic firmware detection
  - Connection testing

---

## 🚀 **Quick Start Guide**

### **Option 1: Windows (Easiest)**

1. **Auto-install everything:**
   ```batch
   install_requirements.bat
   ```

2. **Run the master build system:**
   ```batch
   build_all.bat
   ```

3. **Choose from the menu:**
   - `1` - Setup Environment
   - `5` - Build Everything
   - `4` - Upload to Drone

### **Option 2: Manual Setup**

1. **Install required tools:**
   - CMake: https://cmake.org/download/
   - MinGW: https://www.mingw-w64.org/ (or MSYS2)
   - ARM GCC: https://developer.arm.com/downloads/-/gnu-rm
   - Python 3: https://www.python.org/downloads/

2. **Install Python packages:**
   ```bash
   pip install tkinter pyserial requests tqdm
   ```

3. **Build firmware:**
   ```batch
   # Windows
   build_host.bat          # Test on PC
   build_lpc4330.bat       # Build for drone
   
   # Linux/macOS
   chmod +x build_all.sh
   ./build_all.sh
   ```

---

## 🛠️ **Build Process Explained**

### **Host Testing Build**
- **Purpose:** Test firmware logic on your PC
- **Compiler:** Native GCC/MinGW
- **Output:** `build_host/drone_firmware_test.exe`
- **Features:** 
  - Simulated hardware
  - Console logging
  - Safety system testing

### **LPC4330 Target Build**
- **Purpose:** Real drone firmware
- **Compiler:** ARM GCC (`arm-none-eabi-gcc`)
- **Outputs:**
  - `drone_firmware.elf` - Debug executable
  - `drone_firmware.hex` - Intel HEX (most programmers)
  - `drone_firmware.bin` - Binary (DFU/bootloaders)

---

## 📡 **Programming Your Drone**

### **Method 1: GUI Uploader (Recommended)**
```bash
python firmware_uploader.py
```

**Features:**
- **Auto-detects** firmware files
- **Multiple interfaces:** OpenOCD, DFU, Serial bootloader
- **Progress tracking** with real-time logs  
- **Connection testing** before upload
- **Settings saved** between sessions

### **Method 2: Command Line**
```batch
# Windows
program_lpc4330.bat

# Linux  
openocd -f interface/stlink.cfg -f target/lpc4350.cfg -c "program build_lpc4330/drone_firmware.hex verify reset exit"
```

### **Method 3: Direct OpenOCD**
```bash
openocd -f openocd_lpc4330.cfg -c "init; reset halt; flash write_image erase drone_firmware.hex; verify_image drone_firmware.hex; reset run; shutdown"
```

---

## 🔌 **Programming Hardware Support**

### **JTAG/SWD Debuggers**
- **ST-Link V2/V3** (recommended)
- **J-Link** (professional)
- **CMSIS-DAP** compatible
- **Generic FTDI** adapters

### **Connections (ST-Link)**
```
ST-Link  →  LPC4330
GND      →  GND
3.3V     →  VDD (optional)
SWDIO    →  SWDIO (P2_2)
SWCLK    →  SWCLK (P2_0)
NRST     →  RESET (optional)
```

### **Bootloader Methods**
- **DFU Mode** - USB Device Firmware Update
- **Serial Bootloader** - UART-based programming
- **ISP Mode** - In-System Programming via UART

---

## 📊 **GUI Uploader Features**

### **Main Interface**
- **Firmware Selection:** Browse or auto-detect
- **Programmer Choice:** OpenOCD, DFU, Serial, LPCScrypt
- **Progress Bar:** Real-time upload progress
- **Connection Test:** Verify before programming

### **Settings Tab**
- **OpenOCD Interface:** ST-Link, J-Link, CMSIS-DAP
- **Serial Settings:** Port, baud rate configuration
- **Target Selection:** LPC4330/4350/4337

### **Log Tab**
- **Real-time logging** of all operations
- **Save logs** to file for debugging
- **Clear logs** for new sessions

---

## 🔧 **Build Outputs Explained**

### **Host Build (`build_host/`)**
```
drone_firmware_test.exe     # Windows executable
drone_firmware_test         # Linux/macOS executable
```

### **LPC4330 Build (`build_lpc4330/`)**
```
drone_firmware.elf          # Debug executable (GDB)
drone_firmware.hex          # Intel HEX format
drone_firmware.bin          # Raw binary format  
drone_firmware.map          # Memory map file
```

### **File Usage**
- **`.elf`** - For debugging with GDB/OpenOCD
- **`.hex`** - For most programmers (OpenOCD, J-Link, etc.)
- **`.bin`** - For DFU mode and bootloaders
- **`.map`** - Memory usage analysis

---

## 🐛 **Troubleshooting**

### **Build Errors**
```bash
# Check environment
setup_environment.bat       # Windows
./build_all.sh              # Linux/macOS

# Common issues:
# 1. CMake not found → Install CMake
# 2. arm-none-eabi-gcc not found → Install ARM toolchain  
# 3. Make not found → Install MinGW/build-essential
```

### **Programming Errors**
```bash
# Test connection first
python firmware_uploader.py  # Use connection test feature

# OpenOCD issues:
# 1. Check debugger connection
# 2. Verify target power (3.3V)
# 3. Try different adapter speed
# 4. Update OpenOCD configuration
```

### **GUI Issues**
```bash
# Missing Python packages
pip install tkinter pyserial requests tqdm

# Permission issues (Linux)
sudo usermod -a -G dialout $USER  # Add user to dialout group
sudo chmod 666 /dev/ttyUSB*        # Fix USB permissions
```

---

## 📋 **Complete File List**

### **Windows Batch Scripts**
1. `build_all.bat` - Master build menu
2. `build_host.bat` - Host testing build
3. `build_lpc4330.bat` - Target build
4. `setup_environment.bat` - Environment checker
5. `install_requirements.bat` - Auto-installer
6. `program_lpc4330.bat` - Programming script

### **Cross-Platform Scripts**
7. `build_all.sh` - Unix build script
8. `firmware_uploader.py` - GUI upload tool

### **Configuration Files**
9. `openocd_lpc4330.cfg` - Auto-generated OpenOCD config
10. `uploader_config.json` - GUI settings (auto-created)

---

## 🎯 **What You Get**

✅ **Complete build system** for LPC4330 drone firmware  
✅ **Host testing capability** on PC before deployment  
✅ **Professional GUI** for firmware programming  
✅ **Multiple programming methods** (OpenOCD, DFU, Serial)  
✅ **Automatic tool installation** for Windows  
✅ **Cross-platform support** (Windows, Linux, macOS)  
✅ **Progress tracking** and comprehensive logging  
✅ **Connection testing** before programming  
✅ **Auto-detection** of firmware files  
✅ **Settings persistence** between sessions  

---

## 🚁 **Usage Examples**

### **Development Workflow**
```bash
# 1. Build and test
build_all.bat               # Choose option 5 (Build Everything)

# 2. Test on PC first
build_host/drone_firmware_test.exe

# 3. Program drone
python firmware_uploader.py

# 4. Verify operation
# Check drone boot messages via UART
# Test basic functionality
```

### **Production Programming**
```bash
# Batch programming multiple drones
for /L %i in (1,1,10) do (
    echo Programming drone %i
    python firmware_uploader.py --auto --file build_lpc4330/drone_firmware.hex
    pause
)
```

---

**Your complete drone development and programming system is ready!** 🚁✨

Just run `build_all.bat` and choose what you want to do. The system handles everything automatically.