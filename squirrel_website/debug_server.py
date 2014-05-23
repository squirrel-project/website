#!/usr/bin/env python
import SimpleHTTPServer, SocketServer, os, subprocess
from SimpleHTTPServer import SimpleHTTPRequestHandler 
import sys, re

def _notInstalled( program ):
    return not _isInstalled( program )

def _isInstalled( program ):
    cmd = 'type %s > /dev/null 2>&1' % program
    return _exec( cmd, shell=True, silent=True ) == 0

def _exec( cmd, shell=False, silent=False ):
    if not silent:
        print( 'Executing command "%s"' % cmd )
    if not shell:
        cmd = cmd.split( ' ' )
    p = subprocess.Popen( cmd, shell=shell )
    result = p.wait()
    if result != 0 and not silent:
        print( 'Command "%s" failed' % cmd )
    return result


class Handler( SimpleHTTPRequestHandler ):
    def pageExists( self ):
        filepath = '%s/pages/%s' % ( os.getcwd(), self.path )
        return os.path.isfile( filepath )

    def getHeader( self ):
        with file( 'templates/header.tpl', 'r' ) as f:
            content = f.read()
        matcher = re.match( '/?(.*)\.html', self.path )
        if matcher:
            pagename = matcher.group( 1 )
            print 'content pos: %s' % content.find( 'BODYCLASSNAME' )
            content = content.replace( 'BODYCLASSNAME', pagename )
        return content

    def getFooter( self ):
        with file( 'templates/footer.tpl', 'r' ) as f:
            return f.read()

    def getPagecontent( self ):
        with file( 'pages/%s' % self.path, 'r' ) as f:
            return f.read()

    def sendContent( self, content, filetype ):
        self.send_response(200)
        self.send_header("Content-type", filetype )
        self.send_header("Content-length", len( content ))
        self.end_headers()
        self.wfile.write( content )
        self.wfile.close()

    def sendPage( self ):
        content = ''.join([
            self.getHeader(),
            self.getPagecontent(),
            self.getFooter()
        ])
        self.sendContent( content, 'text/html' )


    def compileScss( self ):
        if _notInstalled( 'scss' ):
            print( 'Scss not installed, not building stylesheet' )
            return ''

        filepath = '/tmp/style.css'
        _exec( 'scss style/style.scss:%s' % filepath )
        with file( filepath ) as f:
            return f.read()

    def sendCss( self ):
        print 'Send css'
        content = self.compileScss()
        self.sendContent( content, 'text/css' )
        self.send_header("Content-length", len( content ))

    def do_GET( self ):
        if self.path == '/':
            self.path = '/index.html'

        if( self.pageExists() ):
            self.sendPage()
        elif self.path.endswith( 'style.css' ):
            self.sendCss()
        else:
            SimpleHTTPRequestHandler.do_GET( self )

def cdIntoHTML():
    baseDirectory = os.path.dirname(os.path.realpath(__file__))
    htmlDirectory = '%s/html' % baseDirectory
    os.chdir( htmlDirectory )

if __name__ == '__main__':
    cdIntoHTML()
    port = int(sys.argv[1])

    # Check if scss is installed
    if _notInstalled( 'scss' ):
        print( 'Scss not installed, not building stylesheet' )

    # Start server
    SocketServer.TCPServer.allow_reuse_address = True
    httpd = SocketServer.TCPServer(( '', port ), Handler )
    print 'Serving from port %s' % port
    httpd.serve_forever()
