"""Decode one actual WHEP frame (no headset needed), optionally save it."""
import argparse,asyncio
from pathlib import Path
from urllib.request import Request,urlopen
from aiortc import RTCPeerConnection,RTCSessionDescription

async def run(url,output):
    peer=RTCPeerConnection();peer.addTransceiver('video',direction='recvonly')
    frame=asyncio.get_running_loop().create_future()
    @peer.on('track')
    def track(track):
        async def read():
            try:
                image=await track.recv()
                if not frame.done():frame.set_result(image)
            except Exception as exc:
                if not frame.done():frame.set_exception(exc)
        asyncio.create_task(read())
    try:
        await peer.setLocalDescription(await peer.createOffer())
        request=Request(url,peer.localDescription.sdp.encode(),{'Content-Type':'application/sdp'},method='POST')
        def offer():
            with urlopen(request,timeout=10) as response:return response.read().decode()
        answer=await asyncio.to_thread(offer)
        await peer.setRemoteDescription(RTCSessionDescription(answer,'answer'))
        image=await asyncio.wait_for(frame,20)
        if output:image.to_image().save(output)
        print(f'WHEP decoded: {image.width}x{image.height}; timestamp={image.pts}')
    finally:await peer.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://127.0.0.1:8889/st/whep');p.add_argument('--output')
    a=p.parse_args();asyncio.run(run(a.url,a.output))
