//
// Created by 4NR_Operator_3 on 21.11.2025.
//

#pragma once
#include "PCH.h"

template<typename InterpolationType>
class IDataUploader {
public:
    IDataUploader() = default;
    virtual ~IDataUploader() = default;
    IDataUploader(const IDataUploader&) = delete;
    IDataUploader& operator=(const IDataUploader&) = default;
    IDataUploader(IDataUploader&&) = default;
    IDataUploader& operator=(IDataUploader&&) = default;
    virtual std::unique_ptr<InterpolationType> loadFromFile() = 0;

};
