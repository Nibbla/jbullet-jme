There is no Interface system to wrap the jbullet-jme classes and so a native version will just be the same classes (in respect to public methods) in a different package.

I did not implement an interface system because of multiple things. First, I do not think that jbullet will be developed with the same pace as bullet is. Things such as softbodys will probably never be implemented if not somebody helps the developer whos porting it to java. So when a native version is started, extending the jbullet version to throw exceptions etc. is probably a bad idea.

The idea is, however, to have the native bullet implementation be api-compatible to the jbullet implementation. This way programs built with jbullet-jme just need to change the package names in the source to get access to the extended possibilities of a native implementation.

There is no need to be able to "hot swap" the different implementations like in jmephysics. If you want the simple, platform-independent phyiscs, you use the jbullet version. If you need extended physics, you use the native version. If you find out in between you just change the package name.