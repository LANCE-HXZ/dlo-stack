from torch import nn
import torch
from torchvision import models
import torchvision
from torch.nn import functional as F

def conv3x3(in_,out):
	return nn.Conv2d(in_, out,3,padding = 1)

class ConvRelu(nn.Module):
	def __init__(self,in_,out):
		super().__init__()
		self.conv = conv3x3(in_,out)
		self.activation = nn.ReLU(inplace = True)
	def forward(self,x):
		x = self.conv(x)
		x = self.activation(x)
		return x

class DecoderBlock(nn.Module):
	def __init__(self, in_channels, middle_channels, out_channels, is_deconv = True):
		super(DecoderBlock,self).__init__()
		self.in_channels = in_channels

		if is_deconv:
			self.block = nn.Sequential(
				ConvRelu(in_channels, middle_channels),
				nn.ConvTranspose2d(middle_channels,out_channels,kernel_size = 4,stride = 2,
					padding = 1),
				nn.ReLU(inplace = True)
				)
		else:
			self.block = nn.Sequential(
				nn.Upsample(scale_factor = 2,mode = 'bilinear'),
				ConvRelu(in_channels, middle_channels),
				ConvRelu(middle_channels, out_channels),
				)
	def forward(self, x):
		return self.block(x)

class UNet16(nn.Module):
	def __init__(self, num_filters = 32, pretrained = False):
		super().__init__()
		self.pool = nn.MaxPool2d(2,2)
		if pretrained == 'vgg':
			self.encoder = torchvision.models.vgg16(pretrained = True).features
		else:
			self.encoder = torchvision.models.vgg16(pretrained = False).features

		self.relu = nn.ReLU(inplace = True)
		# conv0 = nn.Sequential(
		# 	nn.Conv2d(9,64,kernel_size = (3,3),stride = (1,1),padding = (1,1)),
		# 	nn.relu)
		################First Stage Downsample#################
		self.conv1 = nn.Sequential(
			self.encoder[0],
			self.relu,
			self.encoder[2],
			self.relu)	#3->64->64
		self.conv2 = nn.Sequential(
			self.encoder[5],
			self.relu,
			self.encoder[7],
			self.relu)	#64->128->128
		self.conv3 = nn.Sequential(
			self.encoder[10],
			self.relu,
			self.encoder[12],
			self.relu,
			self.encoder[14],
			self.relu)	#128->256->256
		self.conv4 = nn.Sequential(
			self.encoder[17],
			self.relu,
			self.encoder[19],
			self.relu,
			self.encoder[21],
			self.relu)	#256->512->512
		self.conv5 = nn.Sequential(
			self.encoder[24],
			self.relu,
			self.encoder[26],
			self.relu,
			self.encoder[28],
			self.relu)	#512->512->512
		#####################First Stage Upsample Overlap Map#################
		self.center1 = DecoderBlock(512, num_filters*8*2, num_filters*8)		#512->256
		self.dec5 = DecoderBlock(512 + num_filters*8, num_filters*8*2, num_filters*8)	#768->256
		self.dec4 = DecoderBlock(512 + num_filters*8, num_filters*8*2, num_filters*8)	#768->256
		self.dec3 = DecoderBlock(256 + num_filters*8, num_filters*4*2, num_filters*2)	#512->64
		self.dec2 = DecoderBlock(128 + num_filters*2, num_filters*2*2, num_filters)	#192->32
		self.dec1 = ConvRelu(64 + num_filters, num_filters)				#96->32
		self.final1 = nn.Conv2d(num_filters, 3, kernel_size = 3,padding = 1)		#32->3
		###################First Stage Upsample Gradient Map#################
		self.center2 = DecoderBlock(512, num_filters*8*2, num_filters*8)			#512->256
		self.dec10 = DecoderBlock(512 + num_filters*8, num_filters*8*2, num_filters*8)	#768->256
		self.dec9 = DecoderBlock(512 + num_filters*8, num_filters*8*2, num_filters*8)	#768->256
		self.dec8 = DecoderBlock(256 + num_filters*8, num_filters*4*2, num_filters*2)	#512->64
		self.dec7 = DecoderBlock(128 + num_filters*2, num_filters*2*2, num_filters)	#192->32
		self.dec6 = ConvRelu(64 + num_filters, num_filters)				#96->32
		self.final2 = nn.Conv2d(num_filters, 3, kernel_size = 3,padding = 1)		#32->3
		###################Second Stage Upsample#######################
		self.conv6 = nn.Sequential(
			nn.Conv2d(9,64,kernel_size = (3,3),stride = (1,1), padding = (1,1)),
			self.relu,
			self.encoder[2],
			self.relu)	#9->64
		self.conv7 = nn.Sequential(
			self.encoder[5],
			self.relu,
			self.encoder[7],
			self.relu)	#64->128
		self.conv8 = nn.Sequential(
			self.encoder[10],
			self.relu,
			self.encoder[12],
			self.relu,
			self.encoder[14],
			self.relu)	#128->256
		self.conv9 = nn.Sequential(
			self.encoder[17],
			self.relu,
			self.encoder[19],
			self.relu,
			self.encoder[21],
			self.relu)	#256->512
		self.conv10 = nn.Sequential(
			self.encoder[24],
			self.relu,
			self.encoder[26],
			self.relu,
			self.encoder[28],
			self.relu)	#512->512
		#######################Second Stage Downsample#############
		self.center3 = DecoderBlock(512, num_filters*8*2, num_filters*8)				#512->256
		self.dec15 = DecoderBlock(512 + num_filters*8, num_filters*8*2, num_filters*8)		#768->256
		self.dec14 = DecoderBlock(512 + num_filters*8, num_filters*8*2, num_filters*8)		#768->256
		self.dec13 = DecoderBlock(256 + num_filters*8, num_filters*4*2, num_filters*2)		#512->64
		self.dec12 = DecoderBlock(128 + num_filters*2, num_filters*2*2, num_filters)		#192->32
		self.dec11 = ConvRelu(64 + num_filters, num_filters)					#96->32
		self.final3 = nn.Conv2d(num_filters, 3, kernel_size = 3,padding = 1)			#32->3

	def forward(self, x):
		# x -> 128*128*3
		conv1 = self.conv1(x)	#128*128*3 -> 128*128*64
		conv2 = self.conv2(self.pool(conv1))	#128*128*64->64*64*128
		conv3 = self.conv3(self.pool(conv2))	#64*64*128 -> 32*32*256
		conv4 = self.conv4(self.pool(conv3))	#32*32*256->16*16*512
		conv5 = self.conv5(self.pool(conv4))	#16*16*512->8*8*512
		center1 = self.center1(self.pool(conv5))	#8*8*512->Pooling:4*4*512->TR:8 *8*256
		dec5 = self.dec5(torch.cat([center1, conv5],1))	#8*8*(center:256+conv5:512)->8*8*512->16*16*256
		dec4 = self.dec4(torch.cat([dec5, conv4],1))	#16*16*(dec5:256+conv4:512)->16*16*512->32*32*256
		dec3 = self.dec3(torch.cat([dec4,conv3],1))	#32*32*(dec4:256+conv3:256)->32*32*256->64*64*64
		dec2 = self.dec2(torch.cat([dec3,conv2],1))	#64*64*(dec3:64+conv2:128)->64*64*128->128*128*32
		dec1 = self.dec1(torch.cat([dec2,conv1],1))	#128*128*(dec2:32+conv1:64)->128*128*32                                                                               
		center2 = self.center2(self.pool(conv5))
		dec10 = self.dec10(torch.cat([center2,conv5],1))
		dec9 = self.dec9(torch.cat([dec10,conv4],1))
		dec8 = self.dec8(torch.cat([dec9,conv3],1))
		dec7 = self.dec7(torch.cat([dec8,conv2],1))
		dec6 = self.dec6(torch.cat([dec7,conv1],1))
		x_out_G = self.final2(dec6)
		# x_out_O = F.log_softmax(self.final1(dec1),dim = 0)
		x_out_O = self.final1(dec1)
		Second_X = torch.cat([x,x_out_O,x_out_G],1)
		# print(Second_X.shape)
		conv6 = self.conv6(Second_X)
		conv7 = self.conv7(self.pool(conv6))
		conv8 = self.conv8(self.pool(conv7))
		conv9 = self.conv9(self.pool(conv8))
		conv10 = self.conv10(self.pool(conv9))
		center3 = self.center3(self.pool(conv10))
		dec15 = self.dec15(torch.cat([center3,conv10],1))
		dec14 = self.dec14(torch.cat([dec15,conv9],1))
		dec13 = self.dec13(torch.cat([dec14,conv8],1))
		dec12 = self.dec12(torch.cat([dec13,conv7],1))
		dec11 = self.dec11(torch.cat([dec12,conv6],1))
		Second_Y = self.final3(dec11)

		return x_out_O, x_out_G,Second_Y








# net = UNet16()
# print(net)
