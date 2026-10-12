/** Docker log frames contain base64-encoded UTF-8 bytes, not Latin-1 text.
 * A frame may split a multi-byte character; keep one decoder per connection. */
export function createUtf8StreamDecoder() {
    const decoder = new TextDecoder('utf-8');
    return (base64: string): string => {
        const bytes = Uint8Array.from(atob(base64), char => char.charCodeAt(0));
        return decoder.decode(bytes, {stream: true});
    };
}
