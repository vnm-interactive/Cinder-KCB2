/*
* 
* Copyright (c) 2015, Wieden+Kennedy
* Stephen Schieberl
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include "cinder/app/App.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/draw.h"
#include "cinder/params/Params.h"

#include "Kinect2.h"

using namespace ci;

class FaceApp : public app::App 
{
public:
	void							draw() override;
	void							setup() override;
	void							update() override;
private:
	Kinect2::DeviceRef				mDevice;
	std::vector<Kinect2::Face3d>	mFaces3d;
#if 0
	Surface8uRef				    mInfraChannel;
#else
    Channel16uRef                   mInfraChannel;
#endif
	float							mFrameRate;
	bool							mFullScreen;
	params::InterfaceGlRef		    mParams;

    gl::TextureRef                  mTex;
};

#include "cinder/app/RendererGl.h"
#include "cinder/gl/VboMesh.h"

using namespace ci;
using namespace app;
using namespace std;

void FaceApp::draw()
{
	gl::viewport( getWindowSize() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );

	if ( mInfraChannel ) {
		gl::color( Colorf::white() );
		gl::enable( GL_TEXTURE_2D );
        if (!mTex)
            mTex = gl::Texture::create(*mInfraChannel);
        else
            mTex->update(*mInfraChannel);
        gl::draw(mTex, mTex->getBounds(), Rectf(getWindowBounds()));
	
		gl::disable( GL_TEXTURE_2D );
		gl::pushMatrices();
		gl::scale( vec2( getWindowSize() ) / vec2( mInfraChannel->getSize() ) );
		
		for ( auto& face : mFaces3d ) {
            auto orgMesh = face.getMesh();
            if (orgMesh && orgMesh->getNumIndices() > 0) {
                TriMesh* mesh = (TriMesh*)orgMesh->clone();
#if 1
                vec3* v3 = mesh->getPositions<3>();
                for (size_t i = 0; i < mesh->getNumVertices(); ++i) {
                    vec2 pos2d = mDevice->mapCameraToDepth(v3[i]);
                    v3[i] = vec3(pos2d, 0);
                }
#else
				// Map face points to color image
                vector<vec2> v2(mesh->getNumVertices());
				vec3* v3 = mesh->getPositions<3>();
				for ( size_t i = 0; i < mesh->getNumVertices(); ++i ) {
                    v2[i] = mDevice->mapCameraToDepth(v3[i]);
				}

				// Create VBO mesh from TriMesh indices and 2D vertices
				geom::BufferLayout bufferLayout;
				bufferLayout.append( geom::Attrib::POSITION, 2, 0, 0 );
				vector<pair<geom::BufferLayout, gl::VboRef>> vertexArrayBuffers = { 
					make_pair( bufferLayout, gl::Vbo::create( GL_ARRAY_BUFFER, mesh->getNumVertices() * sizeof( vec2 ), (void*)&v2[ 0 ] ) ) 
				};
				gl::VboMeshRef vboMesh = gl::VboMesh::create( 
					mesh->getNumVertices(), 
					mesh->getPrimitive(), 
					vertexArrayBuffers, 
					mesh->getNumIndices(), 
					GL_UNSIGNED_INT, 
					gl::Vbo::create( GL_ELEMENT_ARRAY_BUFFER, mesh->getNumIndices() * sizeof( uint32_t ), (void*)mesh->getIndices().data() ) 
					);
#endif
				//gl::lineWidth( 0.5f );
				gl::enableWireframe();
                gl::color(ColorAf(0.0f, 0.5f, 0.0f, 0.3f));
                gl::draw(*mesh);
				gl::disableWireframe();

                delete mesh;
			}
		}
		
        gl::color(Colorf(1.0f, 0.0f, 0.0f));
		gl::popMatrices();
	}

	mParams->draw();
}

void FaceApp::setup()
{	
	gl::enableAlphaBlending();
	
	mFrameRate		= 0.0f;
	mFullScreen		= false;

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->enableFaceMesh();
    mDevice->enableJointTracking(false);
	mDevice->connectBodyEventHandler( [ & ]( const Kinect2::BodyFrame frame )
	{
	} );
#if 0
	mDevice->connectColorEventHandler( [ & ]( const Kinect2::ColorFrame frame )
	{
		mInfraChannel = frame.getSurface();
	} );
#else
    mDevice->connectInfraredEventHandler([&](const Kinect2::InfraredFrame frame)
    {
        mInfraChannel = frame.getChannel();
    });
#endif
    mDevice->connectFace3dEventHandler([&](const Kinect2::Face3dFrame& frame)
    {
        if (!frame.getFaces().empty()) {
            mFaces3d = frame.getFaces();
        }
    });

	mParams = params::InterfaceGl::create( "Params", ivec2( 230, 130 ) );
	mParams->addParam( "Frame rate",		&mFrameRate,			"", true );
	mParams->addParam( "Full screen",		&mFullScreen ).key( "f" );
	mParams->addButton( "Quit",				[ & ]() { quit(); } ,	"key=q" );
}

void FaceApp::update()
{
	mFrameRate = getAverageFps();

	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}
}

CINDER_APP( FaceApp, RendererGl, []( App::Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 1024, 848 ).title( "Face App" ) );
	settings->setFrameRate( 60.0f );
} )
 