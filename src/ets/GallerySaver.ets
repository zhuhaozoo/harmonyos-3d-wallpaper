/**
 * 保存到系统图库服务（纯静态）
 *
 * 实现方式：photoAccessHelper.getPhotoAccessHelper(context).showAssetsCreationDialog(...)
 *   - 写入沙箱文件后，把文件 uri + PhotoCreationConfig 交给系统"保存到相册"确认弹窗；
 *   - 弹窗返回目标写入 uri，再将沙箱文件内容拷贝过去；
 *   - 全程无需 WRITE_IMAGEVIDEO 权限（该权限为受限权限，三方应用不可申请）。
 */
import { common } from '@kit.AbilityKit';
import { photoAccessHelper } from '@kit.MediaLibraryKit';
import { fileIo } from '@kit.CoreFileKit';

/** 媒体类型 */
export enum MediaKindType {
  IMAGE = 0,
  VIDEO = 1
}

/** 保存输入（showAssetsCreationDialog 以"沙箱源文件"为保存内容来源） */
export interface SaveMediaInput {
  /** 沙箱内源文件绝对路径（内容将被保存到相册） */
  sourcePath: string;
  /** 媒体类型：图片或视频 */
  mediaType: MediaKindType;
  /** 目标格式后缀，如 jpg / png / mp4 */
  ext: string;
  /** 资源标题（图库中显示） */
  title: string;
}

export class GallerySaver {
  /**
   * 调用系统"保存到相册"确认弹窗，将沙箱源文件保存到系统图库。
   * @param context UIAbilityContext
   * @param input    保存输入
   * @returns true=用户确认并成功保存；false=用户取消或失败
   */
  static async saveMedia(context: common.UIAbilityContext, input: SaveMediaInput): Promise<boolean> {
    try {
      const phAccessHelper: photoAccessHelper.PhotoAccessHelper =
        photoAccessHelper.getPhotoAccessHelper(context);

      // 1. 源文件必须已存在于沙箱
      const srcFileUris: string[] = [input.sourcePath];

      // 2. 描述待创建资源的属性
      const creationConfig: photoAccessHelper.PhotoCreationConfig = {
        title: input.title,
        fileNameExtension: input.ext,
        photoType: input.mediaType === MediaKindType.VIDEO
          ? photoAccessHelper.PhotoType.VIDEO
          : photoAccessHelper.PhotoType.IMAGE,
        subtype: photoAccessHelper.PhotoSubtype.DEFAULT,
      };
      const photoCreationConfigs: Array<photoAccessHelper.PhotoCreationConfig> = [creationConfig];

      // 3. 弹出系统保存确认对话框，得到目标写入 uri
      const desFileUris: Array<string> =
        await phAccessHelper.showAssetsCreationDialog(srcFileUris, photoCreationConfigs);
      if (desFileUris.length === 0) {
        console.info('GallerySaver: 用户取消保存');
        return false;
      }

      // 4. 把沙箱源文件内容拷贝到目标 uri（desFileUris[0] 为 file:// uri，openSync 直接接收）
      const srcFile: fileIo.File = fileIo.openSync(input.sourcePath, fileIo.OpenMode.READ_ONLY);
      try {
        const desFile: fileIo.File = fileIo.openSync(desFileUris[0], fileIo.OpenMode.WRITE_ONLY);
        try {
          fileIo.copyFileSync(srcFile.fd, desFile.fd);
        } finally {
          try { fileIo.closeSync(desFile); } catch (e) { /* ignore */ }
        }
      } finally {
        try { fileIo.closeSync(srcFile); } catch (e) { /* ignore */ }
      }

      console.info('GallerySaver: 保存到图库成功 title=' + input.title);
      return true;
    } catch (e) {
      console.error('GallerySaver saveMedia error: ' + JSON.stringify(e));
      return false;
    }
  }
}
