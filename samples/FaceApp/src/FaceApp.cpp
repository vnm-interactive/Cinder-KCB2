#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"

#include "cinder/gl/Texture.h"
#include "cinder/gl/draw.h"
#include "cinder/gl/scoped.h"
#include "cinder/gl/shader.h"
#include "cinder/gl/VboMesh.h"

#include "cinder/params/Params.h"
#include "cinder/qtime/QuickTimeGl.h"

#include "Kinect2.h"

using namespace ci;
using namespace app;
using namespace std;

class FaceApp : public app::App
{
public:
    void							draw() override;
    void							setup() override;
    void							update() override;
private:
    Kinect2::DeviceRef				mDevice;
    std::vector<Kinect2::Face3d>	mFaces3d;
    Channel16uRef                   mInfraChannel;
    float							mFrameRate;
    bool							mFullScreen;
    params::InterfaceGlRef		    mParams;

    gl::TextureRef                  mTex;
    qtime::MovieSurfaceRef          mMovie;
    gl::TextureRef                  mMovieTex;

    bool mWireframe = false;
    bool mShowMovie = true;
    bool mShowBackground = false;
    bool mFlip = false;
    bool first = true;
};

void FaceApp::draw()
{
    gl::viewport(getWindowSize());
    gl::clear();
    gl::setMatricesWindow(getWindowSize());

    if (mInfraChannel) {
        gl::color(Colorf::white());
        if (!mTex)
            mTex = gl::Texture::create(*mInfraChannel);
        else
            mTex->update(*mInfraChannel);

        if (mShowBackground)
        {
            gl::draw(mTex, mTex->getBounds(), Rectf(getWindowBounds()));
        }

        gl::pushMatrices();
        gl::scale(vec2(getWindowSize()) / vec2(mInfraChannel->getSize()));

        static vector<vec2> uv;

        static TriMesh* mesh = nullptr;

        for (auto& face : mFaces3d) {
            auto orgMesh = face.getMesh();
            if (!orgMesh || face.getBounds().x1 == 0)
                continue;

            if (mesh)
                delete mesh;
            mesh = (TriMesh*)orgMesh->clone();

            vec3* v3 = mesh->getPositions<3>();
            for (size_t i = 0; i < mesh->getNumVertices(); ++i) {
                vec2 pos2d = mDevice->mapCameraToDepth(v3[i]);
                v3[i] = vec3(pos2d, 0);
            }

            // TODO: Optimize it
            if (first)
            {
                first = false;

                AxisAlignedBox aabb = mesh->calcBoundingBox();
                vec3 min = aabb.getMin();
                vec3 max = aabb.getMax();
                vec3 length = aabb.getExtents() * 2.0f;
                for (size_t i = 0; i < mesh->getNumVertices(); ++i)
                {
                    uv.push_back(
                    {
                        (v3[i].x - min.x) / length.x,
                        (v3[i].y - min.y) / length.y
                    });
                }
            }

            for (size_t i = 0; i < mesh->getNumVertices(); ++i)
            {
                mesh->appendTexCoord0(uv[i]);
            }
        }

        // Optimize it
        if (mesh)
        {
            if (mShowMovie)
            {
                gl::ScopedGlslProg glsl(gl::getStockShader(gl::ShaderDef().texture()));
                gl::ScopedTextureBind tex(mMovieTex);
                gl::draw(*mesh);
            }

            if (mWireframe)
            {
                gl::enableWireframe();
                gl::ScopedColor color(ColorAf(0.0f, 0.5f, 0.0f, 0.3f));
                gl::draw(*mesh);
                gl::disableWireframe();
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

    mFrameRate = 0.0f;
    mFullScreen = false;

    mDevice = Kinect2::Device::create();
    mDevice->start();
    mDevice->enableFaceMesh();
    mDevice->enableJointTracking(false);

    mDevice->connectInfraredEventHandler([&](const Kinect2::InfraredFrame frame)
    {
        mInfraChannel = frame.getChannel();
    });

    mDevice->connectFace3dEventHandler([&](const Kinect2::Face3dFrame& frame)
    {
        if (!frame.getFaces().empty()) {
            mFaces3d = frame.getFaces();
        }
    });

    mParams = params::InterfaceGl::create("Params", ivec2(230, 130));
    mParams->addParam("Frame rate", &mFrameRate, "", true);
    mParams->addParam("Flip", &mFlip).key("p");
    mParams->addParam("First", &first).key("s");
    mParams->addParam("Wireframe", &mWireframe).key("w");
    mParams->addParam("ShowMovie", &mShowMovie).key("m");
    mParams->addParam("Background", &mShowBackground).key("b");
    mParams->addParam("Full screen", &mFullScreen).key("f");
    mParams->addButton("Quit", [&]() { quit(); }, "key=q");

    fs::path moviePath = getAssetPath("face002.mp4");
    try {
        // load up the movie, set it to loop, and begin playing
        mMovie = qtime::MovieSurface::create(moviePath);
        mMovie->setLoop();
        mMovie->play();
    }
    catch (ci::Exception &exc) {
        console() << "Exception caught trying to load the movie" << std::endl;
        mMovie.reset();
    }
}

void FaceApp::update()
{
    mFrameRate = getAverageFps();

    if (mFullScreen != isFullScreen()) {
        setFullScreen(mFullScreen);
        mFullScreen = isFullScreen();
    }

    if (mMovie->checkNewFrame())
    {
        auto surface = mMovie->getSurface();
        if (!mMovieTex)
        {
            mMovieTex = gl::Texture2d::create(*surface, gl::Texture::Format().loadTopDown());
        }
        else
        {
            mMovieTex->update(*surface);
        }
    }

}

CINDER_APP(FaceApp, RendererGl, [](App::Settings* settings)
{
    settings->prepareWindow(Window::Format().size(1024, 848).title("Face App"));
    settings->setFrameRate(60.0f);
})
