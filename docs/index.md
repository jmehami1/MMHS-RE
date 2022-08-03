## Supplementary materials for "Multi-modal Non-Isotropic Light Source Modelling for Reflectance Estimation in Hyperspectral Imaging "

This page contains dataset and supplementary materials for the paper.

![example](images/illustration.png)

### Implementation of the light source estimation

The github repository providing the implementation of the light source estimation is available [here](https://github.com/jmehami1/MMHS-RE).

### Mitsuba Simulations

We forked the 1st version of [Mitsuba](https://www.mitsuba-renderer.org/index_old.html) to render hyperspectral-like hypercubes and RGB images of a scene.

There are two different branches which need to **cloned**, **compiled** and **built**:

- [Mitsuba RGB simulator](https://github.com/jmehami1/mitsuba/tree/master)
- [Mitsuba Spectral simulator](https://github.com/jmehami1/mitsuba/tree/spectral) 

These should be in separate directories. The spectral version renders hypercubes of 100 channels.

added the hyperspectral image generation (see [here](https://github.com/jmehami1/mitsuba)). The config file for the Mitsuba scenes can be found [here](https://drive.google.com/drive/folders/1SIUlGbyHUFoWXUvZ2eydZz-lOyQSqijd?usp=sharing) .

### Supplementary material for the paper

The supplementary material for the paper is available [here](https://drive.google.com/file/d/1SxRDQslgx4DHqqKqA2xz2Qg-8DNb6T10/view?usp=sharing).
