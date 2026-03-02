# 🛠️ UAV Management Utility (`uav_mutil.py`)

The **UAV management utility** (`uav_mutil.py`) provides a command-line interface to create, organize, and manage UAV simulation packages for **UAV_Sim_PyChrono**.
It automates the process of setting up the required file structure, linking the necessary files, and maintaining UAV configuration consistency.

📍 **Location:**
`UAV_Sim_PyChrono/uav_mutil.py`


---

## 📁 UAV Package Directory Structure

```
UAV_Sim_PyChrono/
└── acsl_pychrono/
    └── uav/
        └── X8/
            ├── __init__.py
            ├── X8.py
            ├── X8_config.yaml
            ├── Controller_Gains/
            │   ├── FunnelMRAC.yaml
            │   ├── HybridMRAC.yaml
            │   ├── HybridTwoLayerMRAC.yaml
            │   ├── MRAC.yaml
            │   ├── NonAdaptiveEBCI.yaml
            │   ├── PID.yaml
            │   └── TwoLayerMRAC.yaml
            └── assets/
                ├── X8_export.py
                └── shapes/
                    ├── body_1_1.obj
                    ├── body_8_1.obj
                    └── body_9_1.obj
```
---

## ⚙️ Command Line Help

```bash
python uav_mutil.py --help
```

```
usage: uav_mutil.py [-h] (--uav_create UAV_CREATE | --uav_rename OLD_NAME NEW_NAME | --uav_delete UAV_DELETE | --uav_list ) [--uav_py UAV_PY] [--config CONFIG] [--gains_folder GAINS_FOLDER] [--uav_chrono_py UAV_CHRONO_PY]
                   [--shapes SHAPES] [--template {X8,QUAD,SIMPLE_QUAD}] [--force] [--base_dir BASE_DIR] [--assets_dir ASSETS_DIR]

Manage UAV packages for acsl_pychrono (create, rename, delete, list).

options:
  -h, --help            show this help message and exit
  --uav_create UAV_CREATE
                        UAV name to create (e.g., X8, QUAD, SIMPLE_QUAD)
  --template {X8,QUAD,SIMPLE_QUAD}  
                        Template if config not provided, default="X8"
  --uav_py UAV_PY       Path to UAV Python class file
  --config CONFIG       Path to UAV YAML config file
  --gains_folder GAINS_FOLDER
                        Path to folder with predefined controller gain YAML files
  --uav_chrono_py UAV_CHRONO_PY
                        Path to PyChrono/SolidWorks export file (.py)
  --shapes SHAPES       Path to .obj shape folder
  --uav_rename OLD_NAME NEW_NAME
                        Rename an existing UAV
  --uav_delete UAV_DELETE
                        Delete a UAV (all folder structures)
  --force               Skip confirmation prompts (for delete)
  --uav_list            List all available UAVs
  --base_dir BASE_DIR   Base directory for UAV code packages, default='acsl_pychrono/uav/'
```

---

## 🚀 Functionalities

The utility currently provides **four main functionalities**:

---

### 1️⃣ Create a UAV (`--uav_create`)

Creates a new UAV package and integrates a UAV 3D model into the simulator.
This command sets up the **necessary directory structure and file links** automatically.
The `--template` flag allows the user to create a UAV from our templates library.
If only the a new name is provided under the `--uav_create` flag, the default UAV template will be used. 
Alternatively, the user can provide all required files manually.

Each UAV model consists of the following components:

| Component                   | Flag                  | Description                                                                                                                                                                              | Target Path                                                      |
| --------------------------- | --------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| **1) UAV Class File**       | **`--uav_py`**        | Python script defining the UAV-specific class (must redefine `_load_inertia(self)`, `_compute_estimated_parameters(self)` and `_compute_mixer_matrix(self, cfg)` methods).               | `UAV_Sim_PyChrono/acsl_pychrono/uav/{UAV_NAME}`                  |
| **2) UAV Config File**      | **`--config`**        | YAML file containing all physical parameters for the UAV in Chrono.                                                                                                                      | `UAV_Sim_PyChrono/acsl_pychrono/uav/{UAV_NAME}`                  |
| **3) Controller Gains**     | **`--gains_folder`**  | Folder containing gain YAMLs for each controller. Gains must follow the expected structure or controllers may not load correctly. Linked via `controller.gain_yaml_files` in the config. | `UAV_Sim_PyChrono/acsl_pychrono/uav/{UAV_NAME}/Controller_Gains` |
| **4) PyChrono Export File** | **`--uav_chrono_py`** | Python script exported from the **SolidWorks Chrono Plugin**, defining 3D components and their physical relationships.                                                                   | `UAV_Sim_PyChrono/assets/vehicles/{UAV_NAME}`                    |
| **5) 3D Shapes**            | **`--shapes`**        | Folder containing `.obj` mesh files for each UAV part (main body, propellers, payload box, etc.).                                                                                        | `UAV_Sim_PyChrono/assets/vehicles/{UAV_NAME}`                    |

---

#### 🧩 Examples

**Create a X8 from provided files**

```bash
python uav_mutil.py --uav_create X8_RED \
  --uav_py ./templates/X8/x8.py \
  --config ./templates/X8/x8_config.yaml \
  --gains_folder ./templates/X8/Controller_Gains
  --uav_chrono_py ./templates/X8/CAD_export/x8copter.py \
  --shapes ./templates/X8/CAD_export/shapes \
```

**Compact example**

```bash
python uav_mutil.py --uav_create X8_RED --uav_py ./templates/X8/x8.py --config ./templates/X8/x8_config.yaml --gains_folder ./templates/X8/Controller_Gains --uav_chrono_py ./templates/X8/CAD_export/x8copter.py --shapes ./templates/X8/CAD_export/shapes
```

**Create a quadrotor from provided files**

```bash
python uav_mutil.py --uav_create QUAD_PURPLE \
  --uav_py ./templates/SIMPLE_QUAD/quad.py \
  --config ./templates/SIMPLE_QUAD/quad_config.yaml \
  --gains_folder ./templates/SIMPLE_QUAD/Controller_Gains
  --uav_chrono_py ./templates/SIMPLE_QUAD/CAD_export/QUAD_export.py \
  --shapes ./templates/SIMPLE_QUAD/CAD_export/shapes \
```

**Compact example**

```bash
python uav_mutil.py --uav_create QUAD_PURPLE --uav_py ./templates/SIMPLE_QUAD/quad.py --config ./templates/SIMPLE_QUAD/quad_config.yaml --gains_folder ./templates/SIMPLE_QUAD/Controller_Gains --uav_chrono_py ./templates/SIMPLE_QUAD/CAD_export/QUAD_export.py --shapes ./templates/SIMPLE_QUAD/CAD_export/shapes
```

**Create UAV from template only (default = X8)**

```bash
python uav_mutil.py --uav_create X8_DEFAULT
```

**Use alternate template**

```bash
python uav_mutil.py --uav_create Q4 --template QUAD
```

---

### 2️⃣ List Available UAVs (`--uav_list `)

Displays all UAVs currently installed in the simulator, along with their status.
The command checks whether the required files and folders are correctly populated.

✅ — present
⚠️ — missing (non-critical)
❌ — missing (critical)

**Example:**

```bash
python uav_mutil.py --uav_list 
```

**Output:**

```
[INFO] Available UAVs:
────────────────────────────────────────
🚁  QUAD1
    ├─ UAV Class: ❌ Missing
    ├─ Config: ✅
    ├─ Controller Gains: ❌ Missing Folder
    └─ Assets (CAD): ❌ Missing Folder

🚁  QUAD_PURPLE
    ├─ UAV Class: ✅
    ├─ Config: ❌ Missing
    ├─ Controller Gains: ❌ Missing gains yaml files
    └─ Assets (CAD): ❌ Missing pychrono export ".py" file

🚁  SIMPLE_QUAD
    ├─ UAV Class: ✅
    ├─ Config: ✅
    ├─ Controller Gains: ✅
    └─ Assets (CAD): ⚠️ Missing shapes folder

🚁  X8
    ├─ UAV Class: ✅
    ├─ Config: ✅
    ├─ Controller Gains: ✅
    └─ Assets (CAD): ✅

────────────────────────────────────────
[INFO] 4 UAV(s) found.
```

---

### 3️⃣ Rename a UAV (`--uav_rename` OLD_NAME NEW_NAME)

Renames an existing UAV, including:

* Updating the folder structure
* Updating class name and references
* Ensuring no naming conflicts occur

**Example:**

```bash
python uav_mutil.py --uav_rename X8_RED X8
```

**Known limitation:**
Due to filesystem behavior (especially on Windows), the rename command **does not support case-only changes directly**.
For example:

✅ Supported:

```bash
python uav_mutil.py --uav_rename X8_RED my_X8
```

❌ Not supported:

```bash
python uav_mutil.py --uav_rename X8_RED x8_red
```

An easy workaround is to rename the uav in a two step process.

✅ Supported:
```bash
python uav_mutil.py --uav_rename X8_RED X8_RED1
python uav_mutil.py --uav_rename X8_RED1 x8_red
```

---

### 4️⃣ Delete a UAV (`--uav_delete`)

Deletes a UAV’s folder structure and its associated assets.
The utility asks for confirmation before deletion, unless `--force` is used.

**Example:**

```bash
python uav_mutil.py --uav_delete X8_DEFAULT
```

Prompt:

```
[WARNING] Are you sure you want to permanently delete UAV 'X8_DEFAULT'? (y/N):
```

**Force deletion (skip confirmation):**

```bash
python uav_mutil.py --uav_delete X8_DEFAULT --force
```

⚠️ **Caution:**
This operation **permanently removes all files** associated with the UAV.

---

## 🧠 Notes & Best Practices

* Always keep **UAV names unique** (case-insensitive).
* The `_load_inertia_from_cad()`, `_compute_estimated_parameters()` and `_compute_mixer_matrix()` methods must be redefined in every UAV class.
* Make sure controller YAML files match the expected structure for each controller.
* Use `--uav_list` often to validate UAV integrity after modifications.
* Use the provided **templates (X8 and QUAD)** as starting points for new designs.

---

# 🛠️ UAV Management Utility (`uav_mutil.py`)

The **UAV management utility** (`uav_mutil.py`) provides a command-line interface to create, organize, and manage UAV simulation packages for **UAV_Sim_PyChrono**.
It automates the process of setting up the required file structure, linking the necessary files, and maintaining UAV configuration consistency.

📍 **Location:**
`UAV_Sim_PyChrono/uav_mutil.py`


---

## 📁 UAV Package Directory Structure

```
UAV_Sim_PyChrono/
└── acsl_pychrono/
    └── uav/
        └── X8/
            ├── __init__.py
            ├── X8.py
            ├── X8_config.yaml
            ├── Controller_Gains/
            │   ├── FunnelMRAC.yaml
            │   ├── HybridMRAC.yaml
            │   ├── HybridTwoLayerMRAC.yaml
            │   ├── MRAC.yaml
            │   ├── NonAdaptiveEBCI.yaml
            │   ├── PID.yaml
            │   └── TwoLayerMRAC.yaml
            └── assets/
                ├── X8_export.py
                └── shapes/
                    ├── body_1_1.obj
                    ├── body_8_1.obj
                    └── body_9_1.obj
```
---

## ⚙️ Command Line Help

```bash
python uav_mutil.py --help
```

```
usage: uav_mutil.py [-h] (--uav_create UAV_CREATE | --uav_rename OLD_NAME NEW_NAME | --uav_delete UAV_DELETE | --uav_list ) [--uav_py UAV_PY] [--config CONFIG] [--gains_folder GAINS_FOLDER] [--uav_chrono_py UAV_CHRONO_PY]
                   [--shapes SHAPES] [--template {X8,QUAD,SIMPLE_QUAD}] [--force] [--base_dir BASE_DIR] [--assets_dir ASSETS_DIR]

Manage UAV packages for acsl_pychrono (create, rename, delete, list).

options:
  -h, --help            show this help message and exit
  --uav_create UAV_CREATE
                        UAV name to create (e.g., X8, QUAD, SIMPLE_QUAD)
  --template {X8,QUAD,SIMPLE_QUAD}  
                        Template if config not provided, default="X8"
  --uav_py UAV_PY       Path to UAV Python class file
  --config CONFIG       Path to UAV YAML config file
  --gains_folder GAINS_FOLDER
                        Path to folder with predefined controller gain YAML files
  --uav_chrono_py UAV_CHRONO_PY
                        Path to PyChrono/SolidWorks export file (.py)
  --shapes SHAPES       Path to .obj shape folder
  --uav_rename OLD_NAME NEW_NAME
                        Rename an existing UAV
  --uav_delete UAV_DELETE
                        Delete a UAV (all folder structures)
  --force               Skip confirmation prompts (for delete)
  --uav_list            List all available UAVs
  --base_dir BASE_DIR   Base directory for UAV code packages, default='acsl_pychrono/uav/'
```

---

## 🚀 Functionalities

The utility currently provides **four main functionalities**:

---

### 1️⃣ Create a UAV (`--uav_create`)

Creates a new UAV package and integrates a UAV 3D model into the simulator.
This command sets up the **necessary directory structure and file links** automatically.
The `--template` flag allows the user to create a UAV from our templates library.
If only the a new name is provided under the `--uav_create` flag, the default UAV template will be used. 
Alternatively, the user can provide all required files manually.

Each UAV model consists of the following components:

| Component                   | Flag                  | Description                                                                                                                                                                              | Target Path                                                      |
| --------------------------- | --------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| **1) UAV Class File**       | **`--uav_py`**        | Python script defining the UAV-specific class (must redefine `_load_inertia(self)`, `_compute_estimated_parameters(self)` and `_compute_mixer_matrix(self, cfg)` methods).               | `UAV_Sim_PyChrono/acsl_pychrono/uav/{UAV_NAME}`                  |
| **2) UAV Config File**      | **`--config`**        | YAML file containing all physical parameters for the UAV in Chrono.                                                                                                                      | `UAV_Sim_PyChrono/acsl_pychrono/uav/{UAV_NAME}`                  |
| **3) Controller Gains**     | **`--gains_folder`**  | Folder containing gain YAMLs for each controller. Gains must follow the expected structure or controllers may not load correctly. Linked via `controller.gain_yaml_files` in the config. | `UAV_Sim_PyChrono/acsl_pychrono/uav/{UAV_NAME}/Controller_Gains` |
| **4) PyChrono Export File** | **`--uav_chrono_py`** | Python script exported from the **SolidWorks Chrono Plugin**, defining 3D components and their physical relationships.                                                                   | `UAV_Sim_PyChrono/assets/vehicles/{UAV_NAME}`                    |
| **5) 3D Shapes**            | **`--shapes`**        | Folder containing `.obj` mesh files for each UAV part (main body, propellers, payload box, etc.).                                                                                        | `UAV_Sim_PyChrono/assets/vehicles/{UAV_NAME}`                    |

---

#### 🧩 Examples

**Create a X8 from provided files**

```bash
python uav_mutil.py --uav_create X8_RED \
  --uav_py ./templates/X8/x8.py \
  --config ./templates/X8/x8_config.yaml \
  --gains_folder ./templates/X8/Controller_Gains
  --uav_chrono_py ./templates/X8/CAD_export/x8copter.py \
  --shapes ./templates/X8/CAD_export/shapes \
```

**Compact example**

```bash
python uav_mutil.py --uav_create X8_RED --uav_py ./templates/X8/x8.py --config ./templates/X8/x8_config.yaml --gains_folder ./templates/X8/Controller_Gains --uav_chrono_py ./templates/X8/CAD_export/x8copter.py --shapes ./templates/X8/CAD_export/shapes
```

**Create a quadrotor from provided files**

```bash
python uav_mutil.py --uav_create QUAD_PURPLE \
  --uav_py ./templates/SIMPLE_QUAD/quad.py \
  --config ./templates/SIMPLE_QUAD/quad_config.yaml \
  --gains_folder ./templates/SIMPLE_QUAD/Controller_Gains
  --uav_chrono_py ./templates/SIMPLE_QUAD/CAD_export/QUAD_export.py \
  --shapes ./templates/SIMPLE_QUAD/CAD_export/shapes \
```

**Compact example**

```bash
python uav_mutil.py --uav_create QUAD_PURPLE --uav_py ./templates/SIMPLE_QUAD/quad.py --config ./templates/SIMPLE_QUAD/quad_config.yaml --gains_folder ./templates/SIMPLE_QUAD/Controller_Gains --uav_chrono_py ./templates/SIMPLE_QUAD/CAD_export/QUAD_export.py --shapes ./templates/SIMPLE_QUAD/CAD_export/shapes
```

**Create UAV from template only (default = X8)**

```bash
python uav_mutil.py --uav_create X8_DEFAULT
```

**Use alternate template**

```bash
python uav_mutil.py --uav_create Q4 --template QUAD
```

---

### 2️⃣ List Available UAVs (`--uav_list `)

Displays all UAVs currently installed in the simulator, along with their status.
The command checks whether the required files and folders are correctly populated.

✅ — present
⚠️ — missing (non-critical)
❌ — missing (critical)

**Example:**

```bash
python uav_mutil.py --uav_list 
```

**Output:**

```
[INFO] Available UAVs:
────────────────────────────────────────
🚁  QUAD1
    ├─ UAV Class: ❌ Missing
    ├─ Config: ✅
    ├─ Controller Gains: ❌ Missing Folder
    └─ Assets (CAD): ❌ Missing Folder

🚁  QUAD_PURPLE
    ├─ UAV Class: ✅
    ├─ Config: ❌ Missing
    ├─ Controller Gains: ❌ Missing gains yaml files
    └─ Assets (CAD): ❌ Missing pychrono export ".py" file

🚁  SIMPLE_QUAD
    ├─ UAV Class: ✅
    ├─ Config: ✅
    ├─ Controller Gains: ✅
    └─ Assets (CAD): ⚠️ Missing shapes folder

🚁  X8
    ├─ UAV Class: ✅
    ├─ Config: ✅
    ├─ Controller Gains: ✅
    └─ Assets (CAD): ✅

────────────────────────────────────────
[INFO] 4 UAV(s) found.
```

---

### 3️⃣ Rename a UAV (`--uav_rename` OLD_NAME NEW_NAME)

Renames an existing UAV, including:

* Updating the folder structure
* Updating class name and references
* Ensuring no naming conflicts occur

**Example:**

```bash
python uav_mutil.py --uav_rename X8_RED X8
```

**Known limitation:**
Due to filesystem behavior (especially on Windows), the rename command **does not support case-only changes directly**.
For example:

✅ Supported:

```bash
python uav_mutil.py --uav_rename X8_RED my_X8
```

❌ Not supported:

```bash
python uav_mutil.py --uav_rename X8_RED x8_red
```

An easy workaround is to rename the uav in a two step process.

✅ Supported:
```bash
python uav_mutil.py --uav_rename X8_RED X8_RED1
python uav_mutil.py --uav_rename X8_RED1 x8_red
```

---

### 4️⃣ Delete a UAV (`--uav_delete`)

Deletes a UAV’s folder structure and its associated assets.
The utility asks for confirmation before deletion, unless `--force` is used.

**Example:**

```bash
python uav_mutil.py --uav_delete X8_DEFAULT
```

Prompt:

```
[WARNING] Are you sure you want to permanently delete UAV 'X8_DEFAULT'? (y/N):
```

**Force deletion (skip confirmation):**

```bash
python uav_mutil.py --uav_delete X8_DEFAULT --force
```

⚠️ **Caution:**
This operation **permanently removes all files** associated with the UAV.

---

## 🧠 Notes & Best Practices

* Always keep **UAV names unique** (case-insensitive).
* The `_load_inertia_from_cad()`, `_compute_estimated_parameters()` and `_compute_mixer_matrix()` methods must be redefined in every UAV class.
* Make sure controller YAML files match the expected structure for each controller.
* Use `--uav_list` often to validate UAV integrity after modifications.
* Use the provided **templates (X8 and QUAD)** as starting points for new designs.

---
