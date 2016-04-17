## Downloading ##
In the "Downloads" section of this project you will find the most current release version of jbullet-jme


## Installing ##
jbullet-jme consists of several jar libraries that you have to add to your programs classpath.

### Netbeans ###
To add jbullet-jme to your Netbeans project, do the following:
  * Select Tools -> Libraries
  * Press "New Library"
  * Enter "jbullet-jme" as name and press OK
  * Press add jar/folder
  * Select all jar files from the jbullet-jme zip file and press OK
  * Select the "javadoc" tab
  * Press add zip/folder
  * Select the jbullet-jme-javadoc.zip file and press OK
To use the new library in your project:
  * Right-click your project and select Properties
  * Select "Libraries" on the left
  * Press "Add Library"
  * Select jbullet-jme from the list and press OK

## Building ##
If you want the cutting edge of jbullet-jme development, you can build the current svn version yourself.

Using SVN, you can download the most current development version of jbullet-jme like this:
```
svn checkout http://jbullet-jme.googlecode.com/svn/trunk/ jbullet-jme-read-only
```
To build jbullet-jme you have to have at least Java JDK 1.5 and Apache ANT installed. Build, create JavaDoc and run the example like this:
```
cd jbullet-jme-read-only
cd jbullet-jme
ant jar
ant javadoc
ant run
```