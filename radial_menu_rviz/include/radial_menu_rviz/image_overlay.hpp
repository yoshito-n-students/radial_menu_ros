#ifndef RADIAL_MENU_RVIZ_IMAGE_OVERLAY_HPP
#define RADIAL_MENU_RVIZ_IMAGE_OVERLAY_HPP

#include <string>

#include <ros/console.h>

#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgrePixelFormat.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <OGRE/Overlay/OgrePanelOverlayElement.h>

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QSize>

#include <boost/lexical_cast.hpp>

namespace radial_menu_rviz {

class ImageOverlay {
public:
  ImageOverlay() {
    // use the address of this instance as a name suffix to make names unique
    suffix_ = boost::lexical_cast< std::string >(this);

    material_ = Ogre::MaterialManager::getSingleton().create(
        "ImageOverlayPanelMaterial_" + suffix_,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    panel_ = static_cast< Ogre::PanelOverlayElement * >(
        Ogre::OverlayManager::getSingleton().createOverlayElement("Panel",
                                                                  "ImageOverlayPanel_" + suffix_));
    panel_->setMetricsMode(Ogre::GMM_PIXELS); // enable positioning & sizing in pixels
    panel_->setMaterialName(material_->getName());

    overlay_ = Ogre::OverlayManager::getSingleton().create("ImageOverlay_" + suffix_);
    overlay_->add2D(panel_);
  }

  virtual ~ImageOverlay() {
    Ogre::OverlayManager::getSingleton().destroy(overlay_);
    overlay_ = NULL;

    Ogre::OverlayManager::getSingleton().destroyOverlayElement(panel_);
    panel_ = NULL;

    Ogre::MaterialManager::getSingleton().remove(material_->getName());
    material_.setNull();

    if (!texture_.isNull()) {
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
      texture_.setNull();
    }
  }

  void show() { overlay_->show(); }

  void hide() { overlay_->hide(); }

  void setTopLeft(const QPoint &pos) {
    panel_->setPosition(/* left = */ pos.x(), /* top = */ pos.y());
  }

  void setImage(const QImage &unformatted_image) {
    // format the given image if required
    QImage image;
    if (unformatted_image.width() == 0 || unformatted_image.height() == 0) {
      // if the given image is empty, will create a transparent texture
      image = formattedImage(QSize(1, 1), Qt::transparent);
    } else if (unformatted_image.format() != QImage::Format_ARGB32) {
      image = unformatted_image.convertToFormat(QImage::Format_ARGB32);
    } else {
      image = unformatted_image;
    }

    // destroy the current texture if size are diffrent
    // because we cannot change size of texture after creation
    if (!texture_.isNull() &&
        (image.width() != texture_->getWidth() || image.height() != texture_->getHeight())) {
      material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
      texture_.setNull();
    }

    // create a new texture if not exists
    if (texture_.isNull()) {
      texture_ = Ogre::TextureManager::getSingleton().createManual(
          "ImageOverlayPanelMaterialTexture_" + suffix_,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, image.width(),
          image.height(), /* num of mipmaps = */ 0, Ogre::PF_A8R8G8B8);
      // associate the texture & material
      material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_->getName());
      material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      // match size of the texture & panel to show the entire area of texture on the panel
      panel_->setDimensions(texture_->getWidth(), texture_->getHeight());
    }

    // copy pixel data from the given image to the texture
    {
      const Ogre::HardwarePixelBufferSharedPtr buffer(texture_->getBuffer());
      buffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
      std::memcpy(buffer->getCurrentLock().data, image.constBits(), buffer->getSizeInBytes());
      buffer->unlock();
    }
  }

  // helper function to create QImage in the expected format
  static QImage formattedImage(const QSize &size, const QColor &color) {
    QImage image(size, QImage::Format_ARGB32);
    image.fill(color);
    return image;
  }

private:
  std::string suffix_;
  Ogre::Overlay *overlay_;
  Ogre::PanelOverlayElement *panel_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
};

} // namespace radial_menu_rviz

#endif