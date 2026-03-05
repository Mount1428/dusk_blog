---
title: {{ replace .File.ContentBaseName "-" " " | title }}
slug: "博客url地址，也是唯一标识"
published: {{ .Date }}
summary: "文章简介"
cover:
  image: 文章封面图。也支持HTTPS
tags: [标签1, 标签2]
categories: '文章所处的分类'
draft: false
---
