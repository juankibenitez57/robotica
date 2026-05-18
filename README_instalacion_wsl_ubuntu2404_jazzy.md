# Instalación del entorno del curso en WSL con Ubuntu 24.04 + ROS 2 Jazzy

Este documento explica cómo instalar el entorno del curso en **Windows usando WSL2**, con **Ubuntu 24.04 LTS**, **ROS 2 Jazzy**, **Gazebo**, **MoveIt**, **Navigation2**, el entorno Python del curso, **Terminator** y alias sencillos para el día a día.

Está pensado para alumnado con poca experiencia.

---

## 1. Qué instala este entorno

El script instala:

- ROS 2 **Jazzy**.
- Gazebo a través de `ros_gz`.
- Paquetes del curso equivalentes a los del instalador original, pero actualizados a Ubuntu 24.04 / ROS Jazzy.
- Navigation2.
- MoveIt.
- `rmw_cyclonedds_cpp`.
- Un entorno virtual Python en `/opt/ai-venv` con librerías de IA y visión.
- `terminator` para trabajar con varias terminales.
- Alias y scripts de ayuda para:
  - entrar al entorno del curso,
  - salir del entorno,
  - arrancar Gazebo,
  - abrir VS Code,
  - abrir Terminator.

---

## 2. Archivos

- `install_course_env_wsl_ubuntu2404_jazzy.sh`: instalador principal.
- `README_instalacion_wsl_ubuntu2404_jazzy.md`: esta guía.

---

## 3. Requisitos previos en Windows

Antes de ejecutar el script, en Windows debes tener:

### Opción recomendada: instalar WSL desde PowerShell

Abre **PowerShell como administrador** y ejecuta:

```powershell
wsl --install -d Ubuntu-24.04
```

Después reinicia Windows si te lo pide.

### Actualizar WSL

Conviene ejecutar también:

```powershell
wsl --update
```

### Requisito importante para Gazebo y aplicaciones gráficas

Para que Gazebo pueda abrirse usando la parte gráfica de Windows:

- usa **WSL2**,
- usa **WSLg**,
- mantén actualizado Windows,
- instala el driver gráfico más reciente de tu GPU en Windows (Intel, AMD o NVIDIA).

---

## 4. Primer arranque de Ubuntu en WSL

La primera vez que abras Ubuntu:

1. Windows te pedirá crear un **usuario Linux** y una **contraseña**.
2. Esa contraseña será la que uses luego con `sudo`.

Guárdala bien.

---

## 5. Copiar los archivos al sitio correcto

Puedes copiar los archivos del instalador a tu carpeta Linux, por ejemplo:

```bash
mkdir -p ~/curso_ros2
cd ~/curso_ros2
```

Coloca ahí:

- `install_course_env_wsl_ubuntu2404_jazzy.sh`
- `README_instalacion_wsl_ubuntu2404_jazzy.md`

### Recomendación

Es mejor trabajar dentro del sistema Linux, por ejemplo en `~/curso_ros2`, y no dentro de rutas tipo `/mnt/c/...`.

---

## 6. Dar permisos y ejecutar el instalador

Desde Ubuntu en WSL:

```bash
cd ~/curso_ros2
chmod +x install_course_env_wsl_ubuntu2404_jazzy.sh
sudo bash ./install_course_env_wsl_ubuntu2404_jazzy.sh
```

La instalación puede tardar bastante.

---

## 7. Instalación opcional de VS Code dentro de Ubuntu

### Opción A. Recomendación general

Usar **VS Code de Windows** con la extensión **WSL**.

Ventajas:

- experiencia muy buena para la mayoría del alumnado,
- integración directa con el entorno WSL,
- no necesitas otra instalación dentro de Ubuntu.

En este caso, normalmente desde WSL podrás abrir el proyecto con:

```bash
code .
```

### Opción B. Instalar VS Code también dentro de Ubuntu

Si quieres que el script instale la versión Linux de VS Code dentro de Ubuntu, ejecuta:

```bash
sudo INSTALL_VSCODE=1 bash ./install_course_env_wsl_ubuntu2404_jazzy.sh
```

Esto es útil si prefieres lanzar VS Code como aplicación Linux mediante WSLg.

---

## 8. Opciones adicionales del script

### Crear swap dentro de Ubuntu WSL

Por defecto, el script **no crea swap**.

Si quieres crearla:

```bash
sudo USE_SWAP=1 SWAP_SIZE_GB=8 bash ./install_course_env_wsl_ubuntu2404_jazzy.sh
```

---

## 9. Qué hacer al terminar la instalación

Cuando el script termine:

1. Cierra la terminal WSL.
2. Ábrela otra vez.
3. Ejecuta:

```bash
source ~/.bashrc
```

---

## 10. Alias importantes para el alumnado

### Entrar al entorno del curso

```bash
ai-on
```

Esto abre una subterminal con:

- ROS 2 Jazzy cargado,
- entorno virtual Python activado,
- variables del curso listas.

### Salir del entorno del curso

```bash
ai-off
```

Esto cierra esa subterminal y te devuelve a la terminal normal.

### Arrancar Gazebo

```bash
gazebo-on
```

También existe este alias equivalente:

```bash
gz-course
```

### Abrir VS Code fácilmente

```bash
code-course
```

### Abrir Terminator

```bash
term-course
```

También se incluyen estos alias rápidos:

```bash
term2
term3
```

---

## 11. Forma recomendada de trabajo en clase

Para evitar confusiones, se recomienda este flujo:

### Para ROS / Python / prácticas del curso

En una terminal:

```bash
ai-on
```

### Para abrir Gazebo

En otra terminal:

```bash
gazebo-on
```

### Para abrir el proyecto en VS Code

En la carpeta de trabajo:

```bash
code-course
```

### Para abrir más terminales

```bash
term-course
```

---

## 12. Verificación de la instalación

Ejecuta:

```bash
sudo /usr/local/bin/verify_ros2_wsl_env.sh
```

Si todo va bien, comprobará:

- ROS 2,
- Gazebo,
- MoveIt,
- Navigation2,
- Terminator,
- el entorno Python del curso.

---

## 13. Problemas habituales

### Problema: `ai-on` no existe

Ejecuta:

```bash
source ~/.bashrc
```

Si sigue sin aparecer, cierra la terminal y vuelve a abrirla.

### Problema: Gazebo no abre ventana

Comprueba:

1. que estás en **WSL2**,
2. que Windows está actualizado,
3. que `wsl --update` se ha ejecutado en Windows,
4. que tu driver gráfico está actualizado,
5. que estás usando Ubuntu 24.04 en WSL y no una sesión SSH externa sin GUI.

### Problema: `code-course` falla

Eso suele significar que no existe el comando `code`.

Tienes dos caminos:

1. instalar **VS Code en Windows** con la extensión **WSL**, o
2. reinstalar el entorno con:

```bash
sudo INSTALL_VSCODE=1 bash ./install_course_env_wsl_ubuntu2404_jazzy.sh
```

### Problema: falta memoria durante la instalación

Puedes repetir la instalación creando swap:

```bash
sudo USE_SWAP=1 SWAP_SIZE_GB=8 bash ./install_course_env_wsl_ubuntu2404_jazzy.sh
```

---

## 14. Resumen muy corto

Si ya tienes WSL y Ubuntu 24.04 funcionando, normalmente basta con esto:

```bash
mkdir -p ~/curso_ros2
cd ~/curso_ros2
chmod +x install_course_env_wsl_ubuntu2404_jazzy.sh
sudo bash ./install_course_env_wsl_ubuntu2404_jazzy.sh
source ~/.bashrc
ai-on
```

Y luego:

```bash
gazebo-on
```

---

## 15. Recomendación final para docentes

Para minimizar incidencias en el aula:

- pedir a los alumnos que lleguen con **Windows actualizado**,
- pedir que ejecuten antes `wsl --update`,
- verificar que pueden abrir alguna app gráfica sencilla en WSL,
- recomendar el uso de **VS Code en Windows + extensión WSL** como opción por defecto,
- dejar la instalación de **VS Code Linux en Ubuntu** como alternativa opcional.
