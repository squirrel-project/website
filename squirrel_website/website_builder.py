#!/usr/bin/env python
import roslib
roslib.load_manifest( 'squirrel_website' )
import rospy, sys, subprocess, os, tempfile, shutil, datetime, time
from subprocess import PIPE

class CmdException( Exception ):
    def __init__( self, msg, result ):
        message = '%s\n%s\n%s' % ( msg, '-'*30, result )
        Exception.__init__( self, message )

class CmdResult( object ):
    def __init__( self, cmd, returncode, stdout, stderr ):
        self.cmd        = cmd
        self.succeeded  = returncode == 0
        self.failed     = not self.succeeded
        self.returncode = returncode
        self.stdout     = stdout.strip()
        self.stderr     = stderr.strip()

    def assertSucceded( self, errorMsg ):
        if self.failed:
            raise CmdException( errorMsg, self )

    def __str__( self ):
        return '\n'.join([
            'Returncode: %s' % self.returncode,
            'Stdout: %s'     % self.stdout,
            'Stderr: %s'     % self.stderr
        ])

class Builder( object ):
    def __init__( self, repository ):
        self._repository = repository
        self._rootPath   = '%s' % os.path.dirname(os.path.realpath(__file__))
        self._basePath   = '%s/html' % self._rootPath
        self._buildPath  = '%s/build' % self._basePath

    def build( self ):
        self._createBuildDirectory()
        self._readHeaderAndFooter()
        try:
            self.buildAll()
            self.addFilesToRepositry()
            if self.isDirty():
                rospy.loginfo( 'Repository dirty, pushing results' )
                self.commitAndPushChanges()
            else:
                rospy.loginfo( 'Repository clean, nothing changed. Leaving.' )
        finally:
            self.cleanup()

    def _createBuildDirectory( self ):
        # create the local directory in which to store the compiled pages
        shutil.rmtree( self._buildPath, True )
        os.mkdir( self._buildPath )

        self._targetPath = tempfile.mkdtemp()
        rospy.loginfo( 'Target path is %s' % self._targetPath )

        # Clone the build repository into the target folder and remove
        # all exisiting files
        cloneRepository = ' && '.join([
            'git clone %s %s' % ( self._repository, self._targetPath ),
            'cd %s' % self._targetPath,
            'git rm -rf *.html', 
            'git rm -rf images'
        ])
        args = ( self._repository, self._targetPath )
        result = self._exec( cloneRepository, shell=True )
        errorMsg = 'Could not clone target repository "%s" to "%s"' % args
        result.assertSucceded( errorMsg )
        rospy.loginfo( 'Cloned "%s" into "%s"' % args )

    def isDirty( self ):
        cmd = 'cd %s && git status --porcelain' % self._targetPath
        result = self._exec( cmd, shell=True )
        return result.stdout != ''

    def addFilesToRepositry( self ):
        rospy.loginfo( 'Copying files from build diretory to repository' )

        # copy all files from buildDir to targetDir and add them to
        # the repository
        cmd = ' && '.join([
            'cp -r %s/* %s' % ( self._buildPath, self._targetPath ),
            'cd %s' % self._targetPath,
            'git add --all .',
        ])
        result = self._exec( cmd, shell=True )
        result.assertSucceded( 'Could not add files to the repository' )

    def commitAndPushChanges( self ):
        now = datetime.datetime.now()
        msg = 'Website built on %s' % now.strftime( '%d %b %Y, %H:%M' )
        rospy.loginfo( 'Commit message: "%s". Pushing...' % msg )

        cmd = ' && '.join([
            'cd %s' % self._targetPath,
            'git commit -m "%s"' % msg,
            'git push origin master:master'
        ])
        result = self._exec( cmd, shell=True )
        result.assertSucceded( 'Could not push changes' )


    def cleanup( self ):
        shutil.rmtree( self._buildPath, True )
        shutil.rmtree( self._targetPath, True )
 

    def _readHeaderAndFooter( self ):
        with file( '%s/templates/header.tpl' % self._basePath ) as f:
            self._header=f.read()
        with file( '%s/templates/footer.tpl' % self._basePath ) as f:
            self._footer=f.read()

    def _getHeader( self, classname ):
        return self._header.replace( 'BODYCLASSNAME', classname )

    def _notInstalled( self, program ):
        return not self._isInstalled( program )

    def _isInstalled( self, program ):
        cmd = 'type %s > /dev/null 2>&1' % program
        return self._exec( cmd, shell=True, silent=True ).succeeded

    def buildAll( self ):
        self.buildJs()
        self.buildScss()
        self.compactCss()
        self.buildPages()
        self.buildRaw()
        self.copyImages()

    def buildJs( self ):
        if self._notInstalled( 'r.js' ):
            rospy.logwarn( 'r.js not installed. Not building javascript files' )
            return
        src = '%s/build.js' % self._basePath
        self._exec( 'r.js -o %s' % src )

    def buildScss( self ):
        if self._notInstalled( 'scss' ):
            rospy.logwarn( 'scss not installed. Not building scss styles' )
            return
        src    = '%s/style/style.scss' % self._basePath
        target = '%s/style.css'  % self._buildPath
        self._exec( 'scss %s:%s' % ( src, target ))


    def compactCss( self ):
        if self._notInstalled( 'r.js' ) or self._notInstalled( 'scss' ):
            rospy.logwarn( 'r.js or scss not installed. Not compressing/copying css files' )
            return
        src    = '%s/style.css' % self._buildPath
        target = '%s/style.css' % self._buildPath
        self._exec( 'r.js -o cssIn=%s out=%s' % ( src, target ))


    def buildRaw( self ):
        path = '%s/raw' % self._basePath
        files = [ f[:-5] for f in os.listdir( path ) if f[ 0 ] != '.' ]
        for filename in files:
            src    = '%s/pages/%s.html' % ( self._basePath, filename )
            target = '%s/%s.html' % ( self._buildPath, filename )
            shutil.copyfile( src, target )

    def buildPages( self ):
        path = '%s/pages' % self._basePath
        files = [ f[:-5] for f in os.listdir( path ) if f[ -5: ] == '.html' ]
        for filename in files:
            self._buildPage( filename )

    def _buildPage( self, filename ):
        src    = '%s/pages/%s.html' % ( self._basePath, filename )
        target = '%s/%s.html' % ( self._buildPath, filename )
        with file( src, 'r' ) as tpl:
            with file( target, 'w' ) as out:
                out.write( self._getHeader( filename ))
                out.write( tpl.read() )
                out.write( self._footer )

    def copyImages( self ):
        src    = '%s/images' % self._basePath
        target = '%s/images' % self._buildPath
        cmd = 'cp -r %s %s' % ( src, target )
        self._exec( cmd, shell=True )

    def _exec( self, cmd, shell=False, silent=False ):
        if isinstance( cmd, list ):
            cmds = cmd
            results = []
            for cmd in cmds:
                result = self._execSingle( cmd, shell, silent )
                results.append( result )
            return results
        else:
            return self._execSingle( cmd, shell, silent )

    def _execSingle( self, cmd, shell=False, silent=False ):
        if not silent:
            rospy.loginfo( 'Executing command "%s"' % cmd )
        if not shell:
            cmd = cmd.split( ' ' )
        p = subprocess.Popen( cmd, shell=shell, stdout=PIPE, stderr=PIPE )
        ( stdout, stderr ) = p.communicate()
        returncode = p.returncode

        result = CmdResult( cmd, returncode, stdout, stderr )
        if result.failed and not silent:
            rospy.logerr( 'Command "%s" failed' % cmd )
            rospy.logerr( str( result ))
        return result

if __name__ == '__main__':
    rospy.init_node( 'squirrel_website_builder' )
    repository = rospy.get_param( '~repository' )
    Builder( repository ).build()
    rospy.signal_shutdown( 'Build finished' )
