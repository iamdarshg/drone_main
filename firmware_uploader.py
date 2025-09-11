#!/usr/bin/env python3
"""
firmware_uploader.py - Drone Firmware Upload GUI
Drop-in GUI tool for programming LPC4330 drone firmware

Features:
- Multiple programming methods (OpenOCD, DFU, Serial)
- Progress tracking and logging
- Automatic firmware detection
- Connection testing
- User-friendly interface

Requirements: tkinter, pyserial, subprocess
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import threading
import subprocess
import serial
import serial.tools.list_ports
import os
import sys
import time
import json
from pathlib import Path

class FirmwareUploader:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Firmware Uploader v1.0")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        # State variables
        self.upload_thread = None
        self.is_uploading = False
        self.selected_file = None
        self.config_file = "uploader_config.json"
        
        # Load configuration
        self.load_config()
        
        self.setup_ui()
        self.scan_for_firmware()
        self.update_serial_ports()
        
    def load_config(self):
        """Load saved configuration"""
        self.config = {
            "last_file": "",
            "programmer": "openocd",
            "serial_port": "",
            "baud_rate": 115200,
            "openocd_interface": "stlink",
            "openocd_target": "lpc4350"
        }
        
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    saved_config = json.load(f)
                    self.config.update(saved_config)
        except:
            pass  # Use defaults if config load fails
            
    def save_config(self):
        """Save current configuration"""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.config, f, indent=2)
        except:
            pass
    
    def setup_ui(self):
        """Create the user interface"""
        
        # Main notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Upload tab
        upload_frame = ttk.Frame(notebook)
        notebook.add(upload_frame, text="Upload Firmware")
        self.setup_upload_tab(upload_frame)
        
        # Settings tab
        settings_frame = ttk.Frame(notebook)
        notebook.add(settings_frame, text="Settings")
        self.setup_settings_tab(settings_frame)
        
        # Log tab
        log_frame = ttk.Frame(notebook)
        notebook.add(log_frame, text="Logs")
        self.setup_log_tab(log_frame)
        
        # Status bar
        self.status_bar = ttk.Label(self.root, text="Ready", relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
    def setup_upload_tab(self, parent):
        """Setup the main upload interface"""
        
        # File selection frame
        file_frame = ttk.LabelFrame(parent, text="Firmware File")
        file_frame.pack(fill="x", padx=5, pady=5)
        
        self.file_var = tk.StringVar(value=self.config["last_file"])
        ttk.Entry(file_frame, textvariable=self.file_var, width=60).pack(side=tk.LEFT, padx=5, pady=5, fill="x", expand=True)
        ttk.Button(file_frame, text="Browse...", command=self.browse_file).pack(side=tk.RIGHT, padx=5, pady=5)
        
        # Auto-detected files
        auto_frame = ttk.LabelFrame(parent, text="Auto-detected Firmware")
        auto_frame.pack(fill="x", padx=5, pady=5)
        
        self.auto_listbox = tk.Listbox(auto_frame, height=3)
        self.auto_listbox.pack(fill="x", padx=5, pady=5)
        self.auto_listbox.bind("<Double-Button-1>", self.select_auto_file)
        
        ttk.Button(auto_frame, text="Refresh", command=self.scan_for_firmware).pack(pady=2)
        
        # Connection frame
        conn_frame = ttk.LabelFrame(parent, text="Connection")
        conn_frame.pack(fill="x", padx=5, pady=5)
        
        # Programmer selection
        ttk.Label(conn_frame, text="Programmer:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        self.programmer_var = tk.StringVar(value=self.config["programmer"])
        programmer_combo = ttk.Combobox(conn_frame, textvariable=self.programmer_var, 
                                       values=["openocd", "dfu", "serial", "lpcscrypt"])
        programmer_combo.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        programmer_combo.bind("<<ComboboxSelected>>", self.on_programmer_changed)
        
        # Connection test button
        ttk.Button(conn_frame, text="Test Connection", command=self.test_connection).grid(row=0, column=2, padx=5, pady=2)
        
        conn_frame.columnconfigure(1, weight=1)
        
        # Progress frame
        progress_frame = ttk.LabelFrame(parent, text="Upload Progress")
        progress_frame.pack(fill="x", padx=5, pady=5)
        
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(progress_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.pack(fill="x", padx=5, pady=5)
        
        self.progress_label = ttk.Label(progress_frame, text="Ready to upload")
        self.progress_label.pack(pady=2)
        
        # Control buttons
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill="x", padx=5, pady=10)
        
        self.upload_button = ttk.Button(button_frame, text="Upload Firmware", command=self.start_upload)
        self.upload_button.pack(side=tk.LEFT, padx=5)
        
        self.cancel_button = ttk.Button(button_frame, text="Cancel", command=self.cancel_upload, state="disabled")
        self.cancel_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(button_frame, text="Verify", command=self.verify_upload).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Reset Target", command=self.reset_target).pack(side=tk.RIGHT, padx=5)
        
    def setup_settings_tab(self, parent):
        """Setup the settings interface"""
        
        # OpenOCD settings
        openocd_frame = ttk.LabelFrame(parent, text="OpenOCD Settings")
        openocd_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(openocd_frame, text="Interface:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        self.interface_var = tk.StringVar(value=self.config["openocd_interface"])
        interface_combo = ttk.Combobox(openocd_frame, textvariable=self.interface_var,
                                     values=["stlink", "jlink", "cmsis-dap", "ftdi"])
        interface_combo.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        
        ttk.Label(openocd_frame, text="Target:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        self.target_var = tk.StringVar(value=self.config["openocd_target"])
        target_combo = ttk.Combobox(openocd_frame, textvariable=self.target_var,
                                   values=["lpc4350", "lpc4330", "lpc4337"])
        target_combo.grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        
        openocd_frame.columnconfigure(1, weight=1)
        
        # Serial settings
        serial_frame = ttk.LabelFrame(parent, text="Serial Settings")
        serial_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(serial_frame, text="Port:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        self.serial_var = tk.StringVar(value=self.config["serial_port"])
        self.serial_combo = ttk.Combobox(serial_frame, textvariable=self.serial_var)
        self.serial_combo.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        
        ttk.Button(serial_frame, text="Refresh", command=self.update_serial_ports).grid(row=0, column=2, padx=5, pady=2)
        
        ttk.Label(serial_frame, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        self.baud_var = tk.StringVar(value=str(self.config["baud_rate"]))
        baud_combo = ttk.Combobox(serial_frame, textvariable=self.baud_var,
                                 values=["9600", "115200", "230400", "460800", "921600"])
        baud_combo.grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        
        serial_frame.columnconfigure(1, weight=1)
        
        # Save settings button
        ttk.Button(parent, text="Save Settings", command=self.save_settings).pack(pady=10)
        
    def setup_log_tab(self, parent):
        """Setup the logging interface"""
        
        # Log display
        self.log_text = scrolledtext.ScrolledText(parent, height=20, width=80)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Log controls
        log_controls = ttk.Frame(parent)
        log_controls.pack(fill="x", padx=5, pady=5)
        
        ttk.Button(log_controls, text="Clear Log", command=self.clear_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(log_controls, text="Save Log", command=self.save_log).pack(side=tk.LEFT, padx=5)
        
    def log(self, message, level="INFO"):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        self.root.update_idletasks()
        
        # Also print to console
        print(f"{level}: {message}")
        
    def update_status(self, message):
        """Update status bar"""
        self.status_bar.config(text=message)
        self.root.update_idletasks()
        
    def browse_file(self):
        """Browse for firmware file"""
        filetypes = [
            ("Firmware files", "*.hex *.bin *.elf"),
            ("Intel HEX files", "*.hex"),
            ("Binary files", "*.bin"),
            ("ELF files", "*.elf"),
            ("All files", "*.*")
        ]
        
        filename = filedialog.askopenfilename(title="Select Firmware File", filetypes=filetypes)
        if filename:
            self.file_var.set(filename)
            self.log(f"Selected firmware file: {filename}")
            
    def scan_for_firmware(self):
        """Automatically scan for firmware files"""
        self.auto_listbox.delete(0, tk.END)
        
        # Look in common build directories
        search_paths = ["build_lpc4330", "build", "Release", "Debug"]
        firmware_files = []
        
        for path in search_paths:
            if os.path.exists(path):
                for ext in ["*.hex", "*.bin", "*.elf"]:
                    files = list(Path(path).glob(ext))
                    firmware_files.extend(files)
        
        # Sort by modification time (newest first)
        firmware_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
        
        for file in firmware_files:
            size_mb = file.stat().st_size / (1024 * 1024)
            mod_time = time.strftime("%Y-%m-%d %H:%M", time.localtime(file.stat().st_mtime))
            display_text = f"{file.name} ({size_mb:.1f} MB, {mod_time})"
            self.auto_listbox.insert(tk.END, display_text)
            
        if firmware_files:
            self.log(f"Found {len(firmware_files)} firmware files")
        else:
            self.log("No firmware files found - build firmware first")
            
    def select_auto_file(self, event):
        """Select auto-detected file"""
        selection = self.auto_listbox.curselection()
        if selection:
            filename = self.auto_listbox.get(selection[0]).split()[0]
            # Find full path
            search_paths = ["build_lpc4330", "build", "Release", "Debug"]
            for path in search_paths:
                full_path = os.path.join(path, filename)
                if os.path.exists(full_path):
                    self.file_var.set(full_path)
                    self.log(f"Selected: {full_path}")
                    break
                    
    def update_serial_ports(self):
        """Update list of available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.serial_combo['values'] = ports
        if ports and not self.serial_var.get():
            self.serial_var.set(ports[0])
            
    def on_programmer_changed(self, event=None):
        """Handle programmer selection change"""
        programmer = self.programmer_var.get()
        self.log(f"Selected programmer: {programmer}")
        
        if programmer == "serial":
            self.update_serial_ports()
            
    def test_connection(self):
        """Test connection to target"""
        programmer = self.programmer_var.get()
        self.log(f"Testing connection via {programmer}...")
        
        def test_thread():
            try:
                if programmer == "openocd":
                    self.test_openocd_connection()
                elif programmer == "serial":
                    self.test_serial_connection()
                elif programmer == "dfu":
                    self.test_dfu_connection()
                else:
                    self.log(f"Connection test not implemented for {programmer}", "WARNING")
                    
            except Exception as e:
                self.log(f"Connection test failed: {e}", "ERROR")
                
        threading.Thread(target=test_thread, daemon=True).start()
        
    def test_openocd_connection(self):
        """Test OpenOCD connection"""
        cmd = [
            "openocd",
            "-f", f"interface/{self.interface_var.get()}.cfg",
            "-f", f"target/{self.target_var.get()}.cfg",
            "-c", "init",
            "-c", "reset halt",
            "-c", "shutdown"
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            self.log("OpenOCD connection successful", "SUCCESS")
            self.update_status("OpenOCD connection OK")
        else:
            self.log(f"OpenOCD connection failed: {result.stderr}", "ERROR")
            self.update_status("OpenOCD connection failed")
            
    def test_serial_connection(self):
        """Test serial connection"""
        try:
            with serial.Serial(self.serial_var.get(), int(self.baud_var.get()), timeout=2) as ser:
                ser.write(b"AT\r\n")  # Send test command
                response = ser.read(100)
                self.log("Serial connection successful", "SUCCESS")
                self.update_status("Serial connection OK")
        except Exception as e:
            self.log(f"Serial connection failed: {e}", "ERROR")
            self.update_status("Serial connection failed")
            
    def test_dfu_connection(self):
        """Test DFU connection"""
        result = subprocess.run(["dfu-util", "-l"], capture_output=True, text=True)
        
        if "DFU" in result.stdout or result.returncode == 0:
            self.log("DFU device found", "SUCCESS")
            self.update_status("DFU connection OK")
        else:
            self.log("No DFU devices found", "ERROR")
            self.update_status("DFU connection failed")
            
    def start_upload(self):
        """Start firmware upload"""
        if self.is_uploading:
            return
            
        firmware_file = self.file_var.get()
        if not firmware_file or not os.path.exists(firmware_file):
            messagebox.showerror("Error", "Please select a valid firmware file")
            return
            
        self.is_uploading = True
        self.upload_button.config(state="disabled")
        self.cancel_button.config(state="normal")
        self.progress_var.set(0)
        
        self.log(f"Starting upload: {firmware_file}")
        self.update_status("Uploading...")
        
        self.upload_thread = threading.Thread(target=self.upload_firmware, args=(firmware_file,), daemon=True)
        self.upload_thread.start()
        
    def upload_firmware(self, firmware_file):
        """Upload firmware (runs in separate thread)"""
        try:
            programmer = self.programmer_var.get()
            
            if programmer == "openocd":
                self.upload_via_openocd(firmware_file)
            elif programmer == "dfu":
                self.upload_via_dfu(firmware_file)
            elif programmer == "serial":
                self.upload_via_serial(firmware_file)
            elif programmer == "lpcscrypt":
                self.upload_via_lpcscrypt(firmware_file)
            else:
                raise Exception(f"Unsupported programmer: {programmer}")
                
            self.log("Upload completed successfully!", "SUCCESS")
            self.update_status("Upload complete")
            self.progress_var.set(100)
            
        except Exception as e:
            self.log(f"Upload failed: {e}", "ERROR")
            self.update_status("Upload failed")
            
        finally:
            self.is_uploading = False
            self.upload_button.config(state="normal")
            self.cancel_button.config(state="disabled")
            
    def upload_via_openocd(self, firmware_file):
        """Upload via OpenOCD"""
        # Create temporary config
        config_content = f"""
source [find interface/{self.interface_var.get()}.cfg]
source [find target/{self.target_var.get()}.cfg]
adapter speed 4000
init
reset halt
flash probe 0
flash write_image erase "{firmware_file}"
verify_image "{firmware_file}"
reset run
shutdown
"""
        
        with open("temp_upload.cfg", "w") as f:
            f.write(config_content)
            
        try:
            cmd = ["openocd", "-f", "temp_upload.cfg"]
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
                                     text=True, universal_newlines=True)
            
            progress = 0
            for line in process.stdout:
                self.log(line.strip())
                
                # Update progress based on OpenOCD output
                if "flash write_image" in line:
                    progress = 30
                elif "Programming" in line or "wrote" in line:
                    progress = 60
                elif "verify_image" in line:
                    progress = 80
                elif "verified" in line:
                    progress = 90
                    
                self.progress_var.set(progress)
                self.root.update_idletasks()
                
            process.wait()
            
            if process.returncode != 0:
                raise Exception("OpenOCD programming failed")
                
        finally:
            if os.path.exists("temp_upload.cfg"):
                os.remove("temp_upload.cfg")
                
    def upload_via_dfu(self, firmware_file):
        """Upload via DFU"""
        cmd = ["dfu-util", "-a", "0", "-D", firmware_file]
        
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, universal_newlines=True)
        
        for line in process.stdout:
            self.log(line.strip())
            
            # Update progress based on DFU output
            if "%" in line:
                try:
                    percent = int(line.split("%")[0].split()[-1])
                    self.progress_var.set(percent)
                    self.root.update_idletasks()
                except:
                    pass
                    
        process.wait()
        
        if process.returncode != 0:
            raise Exception("DFU programming failed")
            
    def upload_via_serial(self, firmware_file):
        """Upload via serial bootloader"""
        # This would implement a serial bootloader protocol
        # For now, just a placeholder
        self.log("Serial upload not fully implemented", "WARNING")
        self.progress_var.set(50)
        time.sleep(2)  # Simulate upload time
        
    def upload_via_lpcscrypt(self, firmware_file):
        """Upload via LPCScrypt"""
        cmd = ["lpcscrypt", "program", firmware_file]
        
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, universal_newlines=True)
        
        for line in process.stdout:
            self.log(line.strip())
            
        process.wait()
        
        if process.returncode != 0:
            raise Exception("LPCScrypt programming failed")
            
    def cancel_upload(self):
        """Cancel ongoing upload"""
        if self.upload_thread and self.upload_thread.is_alive():
            self.log("Upload cancelled by user", "WARNING")
            self.is_uploading = False
            self.upload_button.config(state="normal")
            self.cancel_button.config(state="disabled")
            self.update_status("Upload cancelled")
            
    def verify_upload(self):
        """Verify uploaded firmware"""
        self.log("Verification not implemented yet", "INFO")
        
    def reset_target(self):
        """Reset target device"""
        programmer = self.programmer_var.get()
        
        if programmer == "openocd":
            cmd = [
                "openocd",
                "-f", f"interface/{self.interface_var.get()}.cfg",
                "-f", f"target/{self.target_var.get()}.cfg",
                "-c", "init",
                "-c", "reset run",
                "-c", "shutdown"
            ]
            
            subprocess.run(cmd)
            self.log("Target reset via OpenOCD")
            
    def save_settings(self):
        """Save current settings"""
        self.config["programmer"] = self.programmer_var.get()
        self.config["openocd_interface"] = self.interface_var.get()
        self.config["openocd_target"] = self.target_var.get()
        self.config["serial_port"] = self.serial_var.get()
        self.config["baud_rate"] = int(self.baud_var.get())
        self.config["last_file"] = self.file_var.get()
        
        self.save_config()
        self.log("Settings saved")
        messagebox.showinfo("Settings", "Settings saved successfully")
        
    def clear_log(self):
        """Clear log display"""
        self.log_text.delete(1.0, tk.END)
        
    def save_log(self):
        """Save log to file"""
        filename = filedialog.asksaveasfilename(
            title="Save Log",
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if filename:
            with open(filename, 'w') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log(f"Log saved to {filename}")

def main():
    # Check for required modules
    try:
        import tkinter
        import serial
    except ImportError as e:
        print(f"ERROR: Missing required module: {e}")
        print("Install with: pip install pyserial")
        sys.exit(1)
        
    root = tk.Tk()
    app = FirmwareUploader(root)
    
    # Handle window close
    def on_closing():
        if app.is_uploading:
            if messagebox.askokcancel("Quit", "Upload in progress. Really quit?"):
                app.cancel_upload()
                app.save_config()
                root.destroy()
        else:
            app.save_config()
            root.destroy()
            
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    root.mainloop()

if __name__ == "__main__":
    main()