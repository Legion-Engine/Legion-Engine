#include <core/data/image.hpp>
#include <core/filesystem/assetimporter.hpp>

namespace args::core
{
    std::vector<math::color> ImageCache::m_nullColors;
    async::readonly_rw_spinlock ImageCache::m_nullLock;

    std::unordered_map<id_type, std::unique_ptr<std::pair<async::readonly_rw_spinlock, image>>> ImageCache::m_images;
    async::readonly_rw_spinlock ImageCache::m_imagesLock;
    std::unordered_map<id_type, std::unique_ptr<std::vector<math::color>>> ImageCache::m_colors;
    async::readonly_rw_spinlock ImageCache::m_colorsLock;

    void image::apply_raw(bool lazyApply)
    {
        {
            async::readwrite_guard guard(ImageCache::m_colorsLock);
            ImageCache::m_colors.erase(m_id);
        }
        if (!lazyApply)
            ImageCache::process_raw(m_id);
    }

    const std::vector<math::color>& image::read_colors()
    {
        return ImageCache::read_colors(m_id);
    }

    size_type image::data_size()
    {
        return m_dataSize;
    }

    math::ivec2 image_handle::size()
    {
        auto [lock, image] = ImageCache::get_raw_image(id);
        async::readonly_guard guard(lock);
        if (image == invalid_image)
            return math::ivec2(0, 0);
        return image.size;
    }

    const std::vector<math::color>& image_handle::read_colors()
    {
        return ImageCache::read_colors(id);
    }

    std::pair<async::readonly_rw_spinlock&, image&> image_handle::get_raw_image()
    {
        return ImageCache::get_raw_image(id);
    }

    const std::vector<math::color>& ImageCache::process_raw(id_type id)
    {
        auto [lock, image] = get_raw_image(id);

        {
            async::readonly_guard guard(lock);
            if (image == invalid_image)
                return m_nullColors;
        }

        std::vector<math::color>* ptr = new std::vector<math::color>();
        auto& output = *ptr;

        {
            async::readonly_guard guard(lock);

            output.reserve(image.size.x * image.size.y);

            byte* start = image.m_pixels;
            byte* end = image.m_pixels + image.m_dataSize;
            size_type channelSize = static_cast<uint>(image.format);
            size_type colorSize = static_cast<int>(image.components) * channelSize;
            for (byte* colorPtr = start; colorPtr < end; colorPtr += colorSize)
            {
                math::color color;

                switch (image.components)
                {
                case image_components::grey:
                {
                    switch (image.format)
                    {
                    case channel_format::eight_bit:
                    {
                        float grayValue = (*colorPtr / 255.f);
                        color.r = grayValue;
                        color.g = grayValue;
                        color.b = grayValue; 
                        color.a = 1.f;
                        break;
                    }
                    case channel_format::sixteen_bit:
                    {
                        float grayValue = (*reinterpret_cast<uint16*>(colorPtr) / 65535.f);
                        color.r = grayValue;
                        color.g = grayValue;
                        color.b = grayValue; 
                        color.a = 1.f;
                        break;
                    }
                    case channel_format::float_hdr:
                    {
                        float grayValue = *reinterpret_cast<float*>(colorPtr);
                        color.r = grayValue;
                        color.g = grayValue;
                        color.b = grayValue; 
                        color.a = 1.f;
                        break;
                    }
                    }
                    break;
                }
                case image_components::grey_alpha:
                {
                    byte* gPtr = colorPtr + channelSize * 0;
                    byte* aPtr = colorPtr + channelSize * 1;

                    switch (image.format)
                    {
                    case channel_format::eight_bit:
                    {
                        float grayValue = (*gPtr / 255.f);
                        color.r = grayValue;
                        color.g = grayValue;
                        color.b = grayValue; 
                        color.a = (*aPtr / 255.f);
                        break;
                    }
                    case channel_format::sixteen_bit:
                    {
                        float grayValue = (*reinterpret_cast<uint16*>(gPtr) / 65535.f);
                        color.r = grayValue;
                        color.g = grayValue;
                        color.b = grayValue; 
                        color.a = (*reinterpret_cast<uint16*>(aPtr) / 65535.f);
                        break;
                    }
                    case channel_format::float_hdr:
                    {
                        float grayValue = *reinterpret_cast<float*>(gPtr);
                        color.r = grayValue;
                        color.g = grayValue;
                        color.b = grayValue; 
                        color.a = *reinterpret_cast<float*>(aPtr);
                        break;
                    }
                    }
                    break;
                }
                case image_components::rgb:
                {
                    byte* rPtr = colorPtr + channelSize * 0;
                    byte* gPtr = colorPtr + channelSize * 1;
                    byte* bPtr = colorPtr + channelSize * 2;

                    switch (image.format)
                    {
                    case channel_format::eight_bit:
                    {
                        color.r = (*rPtr / 255.f);
                        color.g = (*gPtr / 255.f);
                        color.b = (*bPtr / 255.f);
                        color.a = 1.f;
                        break;
                    }
                    case channel_format::sixteen_bit:
                    {
                        color.r = (*reinterpret_cast<uint16*>(rPtr) / 65535.f);
                        color.g = (*reinterpret_cast<uint16*>(gPtr) / 65535.f);
                        color.b = (*reinterpret_cast<uint16*>(bPtr) / 65535.f);
                        color.a = 1.f;
                        break;
                    }
                    case channel_format::float_hdr:
                    {
                        color.r = *reinterpret_cast<float*>(rPtr);
                        color.g = *reinterpret_cast<float*>(gPtr);
                        color.b = *reinterpret_cast<float*>(bPtr);
                        color.a = 1.f;
                        break;
                    }
                    }
                    break;
                }
                case image_components::rgba:
                {
                    byte* rPtr = colorPtr + channelSize * 0;
                    byte* gPtr = colorPtr + channelSize * 1;
                    byte* bPtr = colorPtr + channelSize * 2;
                    byte* aPtr = colorPtr + channelSize * 3;

                    switch (image.format)
                    {
                    case channel_format::eight_bit:
                    {
                        color.r = (*rPtr / 255.f);
                        color.g = (*gPtr / 255.f);
                        color.b = (*bPtr / 255.f);
                        color.a = (*aPtr / 255.f);
                        break;
                    }
                    case channel_format::sixteen_bit:
                    {
                        color.r = (*reinterpret_cast<uint16*>(rPtr) / 65535.f);
                        color.g = (*reinterpret_cast<uint16*>(gPtr) / 65535.f);
                        color.b = (*reinterpret_cast<uint16*>(bPtr) / 65535.f);
                        color.a = (*reinterpret_cast<uint16*>(aPtr) / 65535.f);
                        break;
                    }
                    case channel_format::float_hdr:
                    {
                        color.r = *reinterpret_cast<float*>(rPtr);
                        color.g = *reinterpret_cast<float*>(gPtr);
                        color.b = *reinterpret_cast<float*>(bPtr);
                        color.a = *reinterpret_cast<float*>(aPtr);
                        break;
                    }
                    }
                    break;
                }
                }

                output.push_back(color);
            }
        }

        {
            async::readwrite_guard guard(m_colorsLock);
            m_colors.emplace(std::make_pair(id, std::unique_ptr<std::vector<math::color>>(ptr)));
        }

        return output;
    }

    const std::vector<math::color>& ImageCache::read_colors(id_type id)
    {
        async::readonly_guard guard(m_colorsLock);
        if (!m_colors.count(id))
            return process_raw(id);
        return *m_colors[id];
    }

    std::pair<async::readonly_rw_spinlock&, image&> ImageCache::get_raw_image(id_type id)
    {
        async::readonly_guard guard(m_colorsLock);
        if (!m_images.count(id))
            return std::make_pair(std::ref(m_nullLock), std::ref(invalid_image));
        auto& [lock, image] = *m_images[id];
        return std::make_pair(std::ref(lock), std::ref(image));
    }

    image_handle ImageCache::create_image(const std::string& name, const filesystem::view& file, image_import_settings settings)
    {
        id_type id = nameHash(name);

        {
            async::readonly_guard guard(m_imagesLock);
            if (m_images.count(id))
                return { id };
        }

        if (!file.is_valid() || !file.file_info().is_file)
            return invalid_image_handle;

        auto result = filesystem::AssetImporter::tryLoad<image>(file, settings);

        if (result != common::valid)
            return invalid_image_handle;

        {
            async::readwrite_guard guard(m_imagesLock);
            auto* pair_ptr = new std::pair<async::readonly_rw_spinlock, image>(std::make_pair<async::readonly_rw_spinlock, image>(async::readonly_rw_spinlock(), std::move(result)));
            pair_ptr->second.m_id = id;
            m_images.emplace(std::make_pair(id, std::unique_ptr<std::pair<async::readonly_rw_spinlock, image>>(pair_ptr)));
        }

        return { id };
    }

    image_handle ImageCache::get_handle(const std::string& name)
    {
        id_type id = nameHash(name);
        async::readonly_guard guard(m_imagesLock);
        if (m_images.count(id))
            return { id };
        return invalid_image_handle;
    }

    image_handle ImageCache::get_handle(id_type id)
    {
        async::readonly_guard guard(m_imagesLock);
        if (m_images.count(id))
            return { id };
        return invalid_image_handle;
    }

}
